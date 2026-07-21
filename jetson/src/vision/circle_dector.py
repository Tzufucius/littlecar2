import copy
import itertools
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
from scipy.optimize import least_squares


Point = Tuple[float, float]
EllipseModel = Tuple[Point, Tuple[float, float], float]


@dataclass
class CircleDetectResult:
    found: bool
    center: Optional[Point]
    scale: Optional[float]
    confidence: float
    chamfer_score: float
    radial_score: float
    time_ms: Optional[float] = None
    error_px: Optional[float] = None


class CircleDetector:
    """通过多个同轴椭圆拟合并校正透视误差的同心圆检测器。"""

    DEFAULT_CONFIG = {
        "search_roi": None,
        "nominal_radii": [28.0, 30.5, 35.5, 40.2, 44.9, 49.6],
        "adaptive_block_ratio": 0.16,
        "adaptive_threshold_c": 15,
        "min_contour_area_ratio": 0.0012,
        "center_cluster_ratio": 0.05,
        "ring_dedup_ratio": 0.025,
        "max_fit_rings": 3,
        "min_valid_rings": 2,
        "min_axis_ratio": 0.50,
        "min_ellipse_axis": 10.0,
        "max_ellipse_ratio": 0.95,
        "ransac_iterations": 30,
        "ransac_sample_points": 150,
        "ransac_threshold_ratio": 0.00375,
        "ransac_seed": 20260721,
        "homography_points_per_ring": 10,
        "min_confidence": 0.45,
        "camera_width": None,
        "camera_height": None,
        "camera_warmup_frames": 3,
    }

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        self.config = copy.deepcopy(self.DEFAULT_CONFIG)
        if config is not None:
            self.config.update(config)
        self._refresh_nominal_radii()

    def detect(
        self,
        image_path: Optional[str] = None,
        camera_port: Optional[Any] = None,
        frame: Optional[np.ndarray] = None,
        return_debug: bool = False,
    ):
        """检测同心圆圆心，支持文件、摄像头或内存图像输入。"""
        input_frame = self._load_input(image_path, camera_port, frame)
        result, debug = self._locate(input_frame)

        if image_path is not None and result.center is not None:
            ground_truth = self.parse_center_from_filename(image_path)
            if ground_truth is not None:
                result.error_px = math.dist(result.center, ground_truth)

        return (result, debug) if return_debug else result

    def detect_center(
        self,
        image_path: Optional[str] = None,
        camera_port: Optional[Any] = None,
        frame: Optional[np.ndarray] = None,
    ) -> Optional[Point]:
        """只返回亚像素圆心；检测失败时返回 ``None``。"""
        return self.detect(image_path, camera_port, frame).center

    def update_config(self, config: Dict[str, Any]) -> None:
        """更新检测参数。"""
        self.config.update(config)
        self._refresh_nominal_radii()

    def capture_frame(self, camera_port: Any = 0) -> np.ndarray:
        cap = cv2.VideoCapture(camera_port)
        if not cap.isOpened():
            raise RuntimeError(f"无法打开摄像头：{camera_port}")

        if self.config["camera_width"] is not None:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config["camera_width"])
        if self.config["camera_height"] is not None:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config["camera_height"])

        frame = None
        for _ in range(max(1, int(self.config["camera_warmup_frames"]))):
            ok, frame = cap.read()
            if not ok:
                frame = None
        cap.release()

        if frame is None:
            raise RuntimeError(f"摄像头读取失败：{camera_port}")
        return frame

    def _refresh_nominal_radii(self) -> None:
        radii = np.asarray(self.config["nominal_radii"], dtype=np.float64)
        if radii.ndim != 1 or len(radii) < 2 or np.any(radii <= 0):
            raise ValueError("nominal_radii 必须包含至少两个正数")
        self.nominal_radii = np.sort(radii)

    def _load_input(
        self,
        image_path: Optional[str],
        camera_port: Optional[Any],
        frame: Optional[np.ndarray],
    ) -> np.ndarray:
        if frame is not None:
            if not isinstance(frame, np.ndarray):
                raise TypeError("frame 必须是 numpy.ndarray")
            return frame.copy()
        if image_path is not None:
            image = self._read_image(image_path)
            if image is None:
                raise RuntimeError(f"无法读取图像：{image_path}")
            return image
        if camera_port is not None:
            return self.capture_frame(camera_port)
        raise ValueError("必须提供 image_path、camera_port 或 frame 之一")

    @staticmethod
    def _read_image(image_path: str) -> Optional[np.ndarray]:
        """使用字节流读取路径，兼容 Windows 下的非 ASCII 文件名。"""
        try:
            encoded = np.fromfile(str(image_path), dtype=np.uint8)
        except OSError:
            return None
        return cv2.imdecode(encoded, cv2.IMREAD_COLOR)

    def _locate(self, raw_img: np.ndarray):
        search_img, origin, search_roi = self._clip_roi(raw_img)
        gray = self._to_gray(search_img)
        threshold = self._adaptive_threshold(gray)
        candidates = self._find_ellipse_candidates(threshold)
        cluster = self._select_concentric_cluster(candidates, min(gray.shape[:2]))

        debug = {
            "search_roi_xyxy": search_roi,
            "threshold_img": threshold,
            "candidate_count": len(candidates),
            "rough_center": None,
            "corrected_center": None,
            "matched_indices": [],
            "rings": [],
        }
        if len(cluster) < self.config["min_valid_rings"]:
            return self._failure_result(), debug

        image_min_dim = min(gray.shape[:2])
        rings = self._fit_verified_rings(cluster, image_min_dim)
        if len(rings) < self.config["min_valid_rings"]:
            return self._failure_result(), debug

        ring_indices, scale = self._match_rings(rings)
        rough_center = tuple(map(float, rings[0]["model"][0]))
        corrected_center, homography = self._correct_perspective_center(
            [ring["inliers"] for ring in rings], ring_indices, rough_center, scale
        )
        center = (corrected_center[0] + origin[0], corrected_center[1] + origin[1])
        chamfer_score, radial_score, confidence = self._score_detection(rings, image_min_dim)

        debug.update({
            "rough_center": (rough_center[0] + origin[0], rough_center[1] + origin[1]),
            "corrected_center": center,
            "matched_indices": ring_indices,
            "rings": rings,
            "homography": homography,
            "scale": scale,
        })
        result = CircleDetectResult(
            found=confidence >= self.config["min_confidence"],
            center=center if confidence >= self.config["min_confidence"] else None,
            scale=scale if confidence >= self.config["min_confidence"] else None,
            confidence=confidence,
            chamfer_score=chamfer_score,
            radial_score=radial_score,
        )
        return result, debug

    def _clip_roi(self, image: np.ndarray):
        height, width = image.shape[:2]
        roi = self.config["search_roi"]
        if roi is None:
            return image, (0, 0), (0, 0, width, height)
        x1, y1, x2, y2 = (int(value) for value in roi)
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(width, x2), min(height, y2)
        if x2 <= x1 or y2 <= y1:
            raise ValueError("search_roi 必须与图像有效相交")
        return image[y1:y2, x1:x2].copy(), (x1, y1), (x1, y1, x2, y2)

    @staticmethod
    def _to_gray(image: np.ndarray) -> np.ndarray:
        if image.ndim == 2:
            return image
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    def _adaptive_threshold(self, gray: np.ndarray) -> np.ndarray:
        minimum_dimension = min(gray.shape[:2])
        block_size = max(3, int(minimum_dimension * self.config["adaptive_block_ratio"]) | 1)
        return cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,
            block_size, self.config["adaptive_threshold_c"],
        )

    def _find_ellipse_candidates(self, threshold: np.ndarray) -> List[Dict[str, Any]]:
        height, width = threshold.shape[:2]
        minimum_dimension = min(height, width)
        minimum_area = height * width * self.config["min_contour_area_ratio"]
        contours, _ = cv2.findContours(threshold, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for contour in contours:
            if cv2.contourArea(contour) < minimum_area or len(contour) < 5:
                continue
            _, _, box_width, box_height = cv2.boundingRect(contour)
            aspect_ratio = box_width / max(box_height, 1)
            if not 0.55 < aspect_ratio < 1.85:
                continue
            points = contour.reshape(-1, 2).astype(np.float32)
            model = self._fit_ellipse(points)
            if self._valid_ellipse(model, minimum_dimension):
                candidates.append({"model": model, "points": points})
        return candidates

    @staticmethod
    def _fit_ellipse(points: np.ndarray) -> Optional[EllipseModel]:
        if len(points) < 5:
            return None
        try:
            return cv2.fitEllipse(points)
        except cv2.error:
            return None

    def _valid_ellipse(self, model: Optional[EllipseModel], minimum_dimension: int) -> bool:
        if model is None:
            return False
        (center_x, center_y), (axis_a, axis_b), _ = model
        if not np.isfinite([center_x, center_y, axis_a, axis_b]).all() or min(axis_a, axis_b) <= 0:
            return False
        if max(axis_a, axis_b) > minimum_dimension * self.config["max_ellipse_ratio"]:
            return False
        if min(axis_a, axis_b) < self.config["min_ellipse_axis"]:
            return False
        return min(axis_a, axis_b) / max(axis_a, axis_b) >= self.config["min_axis_ratio"]

    def _select_concentric_cluster(self, candidates: Sequence[Dict[str, Any]], minimum_dimension: int):
        tolerance = minimum_dimension * self.config["center_cluster_ratio"]
        clusters: List[List[Dict[str, Any]]] = []
        for candidate in candidates:
            center = candidate["model"][0]
            for cluster in clusters:
                if math.dist(center, cluster[0]["model"][0]) < tolerance:
                    cluster.append(candidate)
                    break
            else:
                clusters.append([candidate])
        if not clusters:
            return []
        deduplicated = self._deduplicate_rings(max(clusters, key=len), minimum_dimension)
        return deduplicated[-int(self.config["max_fit_rings"]):]

    def _deduplicate_rings(self, cluster: Sequence[Dict[str, Any]], minimum_dimension: int):
        ordered = sorted(cluster, key=lambda item: max(item["model"][1]))
        gap = minimum_dimension * self.config["ring_dedup_ratio"]
        selected = []
        for candidate in ordered:
            if not selected or max(candidate["model"][1]) - max(selected[-1]["model"][1]) > gap:
                selected.append(candidate)
        return selected

    def _fit_verified_rings(self, cluster: Sequence[Dict[str, Any]], minimum_dimension: int):
        threshold = max(1.0, minimum_dimension * self.config["ransac_threshold_ratio"])
        verified = []
        for index, candidate in enumerate(cluster):
            model, inliers = self._ransac_ellipse_fit(candidate["points"], minimum_dimension, threshold, index)
            if model is not None and len(inliers) >= 5:
                verified.append({"model": model, "inliers": candidate["points"][inliers]})
        return verified

    def _ransac_ellipse_fit(self, points: np.ndarray, minimum_dimension: int, threshold: float, seed_offset: int):
        if len(points) < 5:
            return None, np.empty(0, dtype=np.int32)
        step = max(1, len(points) // int(self.config["ransac_sample_points"]))
        evaluation_points = points[::step]
        if len(evaluation_points) < 5:
            return None, np.empty(0, dtype=np.int32)
        rng = np.random.default_rng(int(self.config["ransac_seed"]) + seed_offset)
        best_model, best_count = None, 0
        for _ in range(int(self.config["ransac_iterations"])):
            model = self._fit_ellipse(evaluation_points[rng.choice(len(evaluation_points), 5, replace=False)])
            if not self._valid_ellipse(model, minimum_dimension):
                continue
            count = int(np.count_nonzero(self._ellipse_residuals(evaluation_points, model) < threshold))
            if count > best_count:
                best_model, best_count = model, count
        if best_model is None:
            return None, np.empty(0, dtype=np.int32)
        inliers = np.flatnonzero(self._ellipse_residuals(points, best_model) < threshold)
        model = self._fit_ellipse(points[inliers]) if len(inliers) >= 5 else best_model
        return model, inliers

    @staticmethod
    def _ellipse_residuals(points: np.ndarray, model: EllipseModel) -> np.ndarray:
        (center_x, center_y), (width, height), angle = model
        angle = np.deg2rad(angle)
        cosine, sine = np.cos(angle), np.sin(angle)
        dx, dy = points[:, 0] - center_x, points[:, 1] - center_y
        local_x = dx * cosine + dy * sine
        local_y = -dx * sine + dy * cosine
        semi_x, semi_y = max(width / 2.0, 1e-6), max(height / 2.0, 1e-6)
        normalized_radius = np.hypot(local_x / semi_x, local_y / semi_y)
        point_radius = np.hypot(local_x, local_y)
        return point_radius * np.abs(1.0 - 1.0 / np.maximum(normalized_radius, 1e-6))

    def _match_rings(self, rings: Sequence[Dict[str, Any]]):
        measured = np.array([max(ring["model"][1]) / 2.0 for ring in rings], dtype=np.float64)
        best_indices, best_scale, best_error = [], 1.0, float("inf")
        for indices in itertools.combinations(range(len(self.nominal_radii)), len(measured)):
            nominal = self.nominal_radii[list(indices)]
            scale = float(np.mean(nominal / measured))
            error = float(np.sum(((scale * measured - nominal) / nominal) ** 2))
            if error < best_error:
                best_indices, best_scale, best_error = list(indices), scale, error
        return best_indices, best_scale

    def _correct_perspective_center(self, points_list, ring_indices, rough_center: Point, scale: float):
        sampled = [points[::max(1, len(points) // int(self.config["homography_points_per_ring"]))] for points in points_list]
        center_x, center_y = rough_center
        parameters = np.array([scale, 0.0, -scale * center_x, 0.0, scale, -scale * center_y, 0.0, 0.0])
        try:
            result = least_squares(self._homography_residual, parameters, args=(sampled, ring_indices), method="lm")
            homography = self._homography_from_parameters(result.x)
            inverse = np.linalg.inv(homography)
            center = inverse @ np.array([0.0, 0.0, 1.0])
            return (float(center[0] / center[2]), float(center[1] / center[2])), homography
        except (ValueError, np.linalg.LinAlgError):
            return rough_center, self._homography_from_parameters(parameters)

    def _homography_residual(self, parameters, points_list, ring_indices):
        homography = self._homography_from_parameters(parameters)
        residuals = []
        for points, index in zip(points_list, ring_indices):
            homogeneous = np.column_stack((points, np.ones(len(points))))
            projected = homogeneous @ homography.T
            denominator = np.where(np.abs(projected[:, 2]) < 1e-6, 1e-6, projected[:, 2])
            distances = np.hypot(projected[:, 0] / denominator, projected[:, 1] / denominator)
            residuals.extend(distances - self.nominal_radii[index])
        return np.asarray(residuals)

    @staticmethod
    def _homography_from_parameters(parameters) -> np.ndarray:
        return np.array([
            [parameters[0], parameters[1], parameters[2]],
            [parameters[3], parameters[4], parameters[5]],
            [parameters[6], parameters[7], 1.0],
        ])

    def _score_detection(self, rings: Sequence[Dict[str, Any]], minimum_dimension: int):
        residuals = [np.mean(self._ellipse_residuals(ring["inliers"], ring["model"])) for ring in rings]
        chamfer_score = float(np.mean(residuals))
        centers = np.array([ring["model"][0] for ring in rings], dtype=np.float64)
        center_spread = float(np.mean(np.linalg.norm(centers - centers.mean(axis=0), axis=1)))
        radial_score = float(np.clip(1.0 - center_spread / (minimum_dimension * self.config["center_cluster_ratio"]), 0.0, 1.0))
        ring_score = min(1.0, len(rings) / float(self.config["max_fit_rings"]))
        residual_limit = max(1.0, minimum_dimension * self.config["ransac_threshold_ratio"])
        residual_score = float(np.clip(1.0 - chamfer_score / residual_limit, 0.0, 1.0))
        confidence = float(np.clip(0.45 * residual_score + 0.35 * radial_score + 0.20 * ring_score, 0.0, 1.0))
        return chamfer_score, radial_score, confidence

    @staticmethod
    def _failure_result() -> CircleDetectResult:
        return CircleDetectResult(False, None, None, 0.0, float("inf"), 0.0)

    @staticmethod
    def parse_center_from_filename(image_path: str) -> Optional[Tuple[int, int]]:
        match = re.search(r"\((\d+),\s*(\d+)\)", Path(image_path).name)
        return (int(match.group(1)), int(match.group(2))) if match else None

    def draw_debug_result(self, raw_img, result: CircleDetectResult, debug=None, gt=None):
        """在图像上绘制搜索区域、拟合环与校正圆心。"""
        output = raw_img.copy()
        if debug and debug.get("search_roi_xyxy"):
            x1, y1, x2, y2 = debug["search_roi_xyxy"]
            cv2.rectangle(output, (x1, y1), (x2, y2), (180, 180, 180), 2)
            for ring in debug.get("rings", []):
                (cx, cy), axes, angle = ring["model"]
                cv2.ellipse(output, ((cx + x1, cy + y1), axes, angle), (0, 255, 255), 1, cv2.LINE_AA)
        if debug and debug.get("rough_center"):
            cv2.drawMarker(output, tuple(map(int, debug["rough_center"])), (255, 0, 0), cv2.MARKER_CROSS, 30, 2)
        if result.center is not None:
            cv2.drawMarker(output, tuple(map(int, result.center)), (0, 0, 255), cv2.MARKER_CROSS, 40, 2)
        if gt is not None:
            cv2.drawMarker(output, tuple(map(int, gt)), (0, 255, 0), cv2.MARKER_SQUARE, 30, 2)
        if debug is not None:
            center_text = "None" if result.center is None else f"({result.center[0]:.2f}, {result.center[1]:.2f})"
            labels = [
                f"found={result.found} confidence={result.confidence:.2f}",
                f"center={center_text}",
                f"rings={len(debug.get('rings', []))} matched={debug.get('matched_indices', [])}",
            ]
            for index, text in enumerate(labels):
                position = (12, 24 + index * 20)
                cv2.putText(output, text, position, cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 255), 2, cv2.LINE_AA)
                cv2.putText(output, text, position, cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
        return output
