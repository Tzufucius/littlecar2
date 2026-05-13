import cv2
import re
import math
import time
import copy
import numpy as np
from pathlib import Path
from dataclasses import dataclass
from typing import Optional, Tuple, Dict, Any


@dataclass
class CircleDetectResult:
    found: bool
    center: Optional[Tuple[int, int]]
    scale: Optional[float]
    confidence: float
    chamfer_score: float
    radial_score: float
    time_ms: float
    error_px: Optional[float] = None


class CircleDetector:
    DEFAULT_CONFIG = {
        # =========================
        # 搜索 ROI
        # =========================
        # None 表示全图搜索
        # 可设置为 (x1, y1, x2, y2)
        "search_roi": None,

        # 径向投票后，原图精修 ROI 的额外边距
        "fine_roi_margin": 80,

        # =========================
        # 图案结构参数
        # =========================
        "base_radii": [76, 86, 100, 118, 138, 158],
        "base_radius_gain": 1.5,
        "linestyles": ["solid", "--", "solid", "--", "solid", "--"],

        "real_main_thickness": 6,
        "thin_offset": 2.5,
        "use_thin_aux_line": True,

        "solid_weight": 1.0,
        "dash_weight": 0.65,
        "thin_aux_weight": 0.35,

        # =========================
        # 粗定位：径向投票
        # =========================
        "coarse_factor": 0.25,
        "vote_scale_steps": 7,
        "vote_max_edge_points": 7000,
        "vote_min_edge_points": 80,
        "vote_min_gradient": 8.0,
        "vote_weight_cap": 80.0,
        "vote_blur_sigma": 3.0,
        "vote_min_peak": 1e-6,
        "vote_fast_preprocess": True,

        # =========================
        # scale 估计
        # =========================
        "scale_min": 0.70,
        "scale_max": 1.25,
        "scale_est_steps": 31,
        "scale_min_edge_points": 100,
        "scale_hist_smooth": 5,

        # =========================
        # ROI 稀疏模板精修
        # =========================
        "template_margin": 25,
        "template_edge_thickness": 2,
        "max_sparse_points": 1500,
        "min_valid_sparse_points": 300,

        "refine_window": 10,
        "refine_step": 1,
        "refine_scale_range": 0.035,
        "refine_scale_steps": 7,

        # =========================
        # 图像预处理
        # =========================
        "clahe_limit": 2.5,
        "clahe_grid": (8, 8),

        "bg_sigma": 45,
        "norm_alpha": 1.45,
        "norm_beta": -0.45,
        "norm_gamma": 0,

        # gaussian 更快；bilateral 更稳但慢
        "denoise_method": "gaussian",

        "bilateral_d": 9,
        "bilateral_sigma_color": 65,
        "bilateral_sigma_space": 65,

        "gaussian_ksize": (5, 5),
        "gaussian_sigma": 1.2,

        "canny_low": 65,
        "canny_high": 130,

        # =========================
        # 边缘清理
        # 默认关闭，速度更快
        # =========================
        "remove_long_lines": False,
        "line_thr": 90,
        "line_len": 180,
        "line_gap": 18,
        "line_erase_thickness": 5,

        "remove_small_components": False,
        "min_edge_area": 40,

        # =========================
        # 距离变换
        # =========================
        "distance_mask_size": 3,
        "distance_cap": 20.0,

        # =========================
        # 径向验证
        # =========================
        "min_edge_points_for_radial": 200,
        "angle_bins": 72,
        "radial_band": 4,
        "radial_density_gain": 8.0,
        "radial_good_coverage": 0.28,
        "radial_coverage_weight": 0.75,
        "radial_density_weight": 0.25,

        # =========================
        # 置信度
        # =========================
        "chamfer_excellent": 2.5,
        "chamfer_bad": 9.0,
        "confidence_chamfer_weight": 0.55,
        "confidence_radial_weight": 0.45,

        "min_confidence": 0.35,
        "min_radial_score": 0.25,

        # =========================
        # 摄像头参数
        # =========================
        "camera_width": None,
        "camera_height": None,
        "camera_warmup_frames": 3,
    }

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        self.config = copy.deepcopy(self.DEFAULT_CONFIG)

        if config is not None:
            self.config.update(config)

        self.base_radii = np.array(
            self.config["base_radii"],
            dtype=np.float32
        ) * self.config["base_radius_gain"]

        self.template_cache = {}

    # =========================================================
    # 外部调用接口
    # =========================================================

    def detect(
        self,
        image_path: Optional[str] = None,
        camera_port: Optional[Any] = None,
        frame: Optional[np.ndarray] = None,
        return_debug: bool = False
    ):
        """
        主检测函数。

        三种输入方式：
        1. image_path="xxx.png"
        2. camera_port=0 或 "/dev/video0"
        3. frame=np.ndarray

        默认返回 CircleDetectResult。
        如果 return_debug=True，返回 (result, debug)。
        """
        input_frame = self._load_input(
            image_path=image_path,
            camera_port=camera_port,
            frame=frame
        )

        result, debug = self._locate(input_frame)

        # 如果文件名里有真值坐标，例如 (567, 685).jpg，则计算误差
        if image_path is not None:
            gt = self.parse_center_from_filename(image_path)
            if gt is not None and result.center is not None:
                result.error_px = math.hypot(
                    result.center[0] - gt[0],
                    result.center[1] - gt[1]
                )

        if return_debug:
            return result, debug

        return result

    def detect_center(
        self,
        image_path: Optional[str] = None,
        camera_port: Optional[Any] = None,
        frame: Optional[np.ndarray] = None
    ) -> Optional[Tuple[int, int]]:
        """
        只返回圆心坐标。
        检测失败返回 None。
        """
        result = self.detect(
            image_path=image_path,
            camera_port=camera_port,
            frame=frame,
            return_debug=False
        )
        return result.center

    def update_config(self, config: Dict[str, Any]):
        """
        运行过程中修改参数。
        """
        self.config.update(config)
        self.template_cache.clear()

    # =========================================================
    # 输入读取
    # =========================================================

    def _load_input(
        self,
        image_path: Optional[str],
        camera_port: Optional[Any],
        frame: Optional[np.ndarray]
    ) -> np.ndarray:
        if frame is not None:
            if not isinstance(frame, np.ndarray):
                raise TypeError("frame 必须是 numpy.ndarray")
            return frame.copy()

        if image_path is not None:
            img = cv2.imread(str(image_path))
            if img is None:
                raise RuntimeError(f"无法读取图像：{image_path}")
            return img

        if camera_port is not None:
            return self.capture_frame(camera_port)

        raise ValueError("必须提供 image_path、camera_port 或 frame 之一")

    def capture_frame(self, camera_port=0) -> np.ndarray:
        """
        从 OpenCV 摄像头采集一帧。

        camera_port 可以是：
        0
        1
        '/dev/video0'
        'rtsp://...'
        """
        cap = cv2.VideoCapture(camera_port)

        if not cap.isOpened():
            raise RuntimeError(f"无法打开摄像头：{camera_port}")

        if self.config["camera_width"] is not None:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config["camera_width"])

        if self.config["camera_height"] is not None:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config["camera_height"])

        frame = None
        warmup = int(self.config["camera_warmup_frames"])

        for _ in range(max(1, warmup)):
            ret, frame = cap.read()
            if not ret:
                frame = None

        cap.release()

        if frame is None:
            raise RuntimeError(f"摄像头读取失败：{camera_port}")

        return frame

    # =========================================================
    # 核心检测流程
    # =========================================================

    def _locate(self, raw_img: np.ndarray):
        t0 = time.perf_counter()

        # 1. search ROI，默认全图
        search_img, search_origin, search_roi_xyxy = self._clip_roi_xyxy(
            raw_img,
            self.config["search_roi"]
        )

        # 2. search ROI 低分辨率边缘，用于径向投票
        factor = self.config["coarse_factor"]

        gray_small, denoise_small, edges_small = self._preprocess_edges(
            search_img,
            factor=factor,
            for_vote=True
        )

        # 3. 径向投票粗定位
        center_search_roi, vote_acc, center_small = self._radial_vote_center(
            gray_small,
            edges_small,
            factor=factor
        )

        if center_search_roi is None:
            result = self._build_result(
                center=None,
                scale=None,
                chamfer_score=float("inf"),
                radial_score=0.0,
                t0=t0
            )

            debug = {
                "search_img": search_img,
                "search_origin": search_origin,
                "search_roi_xyxy": search_roi_xyxy,
                "gray_small": gray_small,
                "edges_small": edges_small,
                "vote_acc": vote_acc,
                "center_search_roi": None,
                "center_small": None,
            }

            return result, debug

        # 4. 在低分辨率图中估计 scale
        scale0 = self._estimate_scale_by_radial_hist(
            edges=edges_small,
            center=center_small,
            factor=factor
        )

        if scale0 is None:
            scale0 = (self.config["scale_min"] + self.config["scale_max"]) / 2.0

        # 5. 映射到全图坐标
        center0_full = (
            center_search_roi[0] + search_origin[0],
            center_search_roi[1] + search_origin[1]
        )

        # 6. 裁剪 fine ROI
        fine_roi, fine_origin, fine_roi_xyxy = self._crop_fine_roi(
            raw_img,
            center0_full,
            scale0
        )

        # 7. fine ROI 内做高分辨率边缘和距离变换
        gray_roi, denoise_roi, edges_roi = self._preprocess_edges(
            fine_roi,
            factor=1.0,
            for_vote=False
        )

        dist_roi = self._distance_map_from_edges(edges_roi)

        center_roi0 = (
            center0_full[0] - fine_origin[0],
            center0_full[1] - fine_origin[1]
        )

        # 8. fine ROI 内稀疏模板精修
        center_roi_refined, scale_refined, chamfer_score = self._refine_local_roi(
            dist_roi,
            center_roi0,
            scale0
        )

        if center_roi_refined is None:
            center_full = None
            radial_score = 0.0
        else:
            center_full = (
                center_roi_refined[0] + fine_origin[0],
                center_roi_refined[1] + fine_origin[1]
            )

            radial_score = self._radial_verify(
                edges_roi,
                center_roi_refined,
                scale_refined
            )

        result = self._build_result(
            center=center_full,
            scale=scale_refined,
            chamfer_score=chamfer_score,
            radial_score=radial_score,
            t0=t0
        )

        debug = {
            "search_img": search_img,
            "search_origin": search_origin,
            "search_roi_xyxy": search_roi_xyxy,

            "gray_small": gray_small,
            "edges_small": edges_small,
            "vote_acc": vote_acc,
            "center_search_roi": center_search_roi,
            "center_small": center_small,
            "center0_full": center0_full,
            "scale0": scale0,

            "fine_roi": fine_roi,
            "fine_origin": fine_origin,
            "fine_roi_xyxy": fine_roi_xyxy,
            "gray_roi": gray_roi,
            "edges_roi": edges_roi,
            "dist_roi": dist_roi,
            "center_roi0": center_roi0,
            "center_roi_refined": center_roi_refined,
            "scale_refined": scale_refined,
            "chamfer_score": chamfer_score,
            "radial_score": radial_score,
        }

        return result, debug

    # =========================================================
    # 预处理
    # =========================================================

    def _preprocess_edges(self, img, factor=1.0, for_vote=False):
        if factor != 1.0:
            work = cv2.resize(
                img,
                None,
                fx=factor,
                fy=factor,
                interpolation=cv2.INTER_AREA
            )
        else:
            work = img

        gray = cv2.cvtColor(work, cv2.COLOR_BGR2GRAY)

        clahe = cv2.createCLAHE(
            clipLimit=self.config["clahe_limit"],
            tileGridSize=self.config["clahe_grid"]
        )
        gray_clahe = clahe.apply(gray)

        bg_sigma = max(3.0, self.config["bg_sigma"] * factor)
        bg = cv2.GaussianBlur(
            gray_clahe,
            (0, 0),
            sigmaX=bg_sigma,
            sigmaY=bg_sigma
        )

        gray_norm = cv2.addWeighted(
            gray_clahe,
            self.config["norm_alpha"],
            bg,
            self.config["norm_beta"],
            self.config["norm_gamma"]
        )
        gray_norm = np.clip(gray_norm, 0, 255).astype(np.uint8)

        if for_vote and self.config["vote_fast_preprocess"]:
            gray_denoise = cv2.GaussianBlur(gray_norm, (5, 5), 1.0)
        else:
            if self.config["denoise_method"] == "bilateral":
                gray_denoise = cv2.bilateralFilter(
                    gray_norm,
                    d=self.config["bilateral_d"],
                    sigmaColor=self.config["bilateral_sigma_color"],
                    sigmaSpace=self.config["bilateral_sigma_space"]
                )
            else:
                gray_denoise = cv2.GaussianBlur(
                    gray_norm,
                    self.config["gaussian_ksize"],
                    self.config["gaussian_sigma"]
                )

        edges = cv2.Canny(
            gray_denoise,
            self.config["canny_low"],
            self.config["canny_high"]
        )

        if self.config["remove_long_lines"] and not for_vote:
            edges, _ = self._remove_long_lines(edges, factor=factor)

        if self.config["remove_small_components"] and not for_vote:
            edges = self._remove_small_components(edges, factor=factor)

        return gray_norm, gray_denoise, edges

    def _remove_long_lines(self, edges, factor=1.0):
        line_len = max(20, int(self.config["line_len"] * factor))
        line_gap = max(5, int(self.config["line_gap"] * factor))
        line_thr = max(20, int(self.config["line_thr"] * factor))
        erase_thick = max(1, int(self.config["line_erase_thickness"] * factor))

        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=line_thr,
            minLineLength=line_len,
            maxLineGap=line_gap
        )

        line_mask = np.zeros_like(edges)

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                length = math.hypot(x2 - x1, y2 - y1)

                if length >= line_len:
                    cv2.line(
                        line_mask,
                        (x1, y1),
                        (x2, y2),
                        255,
                        thickness=erase_thick
                    )

        cleaned = cv2.bitwise_and(edges, cv2.bitwise_not(line_mask))
        return cleaned, line_mask

    def _remove_small_components(self, edges, factor=1.0):
        clean = edges.copy()
        min_area = max(3, int(self.config["min_edge_area"] * factor * factor))

        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
            clean,
            connectivity=8
        )

        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]

            if area < min_area:
                clean[labels == i] = 0

        return clean

    def _distance_map_from_edges(self, edges):
        inv_edges = 255 - edges

        dist = cv2.distanceTransform(
            inv_edges,
            cv2.DIST_L2,
            self.config["distance_mask_size"]
        )

        dist = np.minimum(dist, self.config["distance_cap"])
        return dist.astype(np.float32)

    # =========================================================
    # ROI
    # =========================================================

    def _clip_roi_xyxy(self, img, roi):
        h, w = img.shape[:2]

        if roi is None:
            return img, (0, 0), (0, 0, w, h)

        x1, y1, x2, y2 = roi

        x1 = max(0, min(w - 1, int(x1)))
        y1 = max(0, min(h - 1, int(y1)))
        x2 = max(x1 + 1, min(w, int(x2)))
        y2 = max(y1 + 1, min(h, int(y2)))

        return img[y1:y2, x1:x2].copy(), (x1, y1), (x1, y1, x2, y2)

    def _crop_fine_roi(self, raw_img, center_full, scale):
        h, w = raw_img.shape[:2]
        cx, cy = center_full

        max_r = max(self.base_radii) * scale
        half = int(max_r + self.config["fine_roi_margin"])

        x1 = max(0, cx - half)
        y1 = max(0, cy - half)
        x2 = min(w, cx + half)
        y2 = min(h, cy + half)

        roi = raw_img[y1:y2, x1:x2].copy()
        return roi, (x1, y1), (x1, y1, x2, y2)

    # =========================================================
    # 径向投票粗定位
    # =========================================================

    def _radial_vote_center(self, gray, edges, factor):
        h, w = edges.shape[:2]

        gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
        gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)

        ys, xs = np.where(edges > 0)

        if len(xs) < self.config["vote_min_edge_points"]:
            return None, None, None

        gxs = gx[ys, xs]
        gys = gy[ys, xs]
        mag = np.sqrt(gxs * gxs + gys * gys)

        valid_grad = mag > self.config["vote_min_gradient"]

        xs = xs[valid_grad]
        ys = ys[valid_grad]
        gxs = gxs[valid_grad]
        gys = gys[valid_grad]
        mag = mag[valid_grad]

        if len(xs) < self.config["vote_min_edge_points"]:
            return None, None, None

        max_points = self.config["vote_max_edge_points"]

        if len(xs) > max_points:
            idx = np.argpartition(mag, -max_points)[-max_points:]
            xs = xs[idx]
            ys = ys[idx]
            gxs = gxs[idx]
            gys = gys[idx]
            mag = mag[idx]

        norm = mag + 1e-6
        ux = gxs / norm
        uy = gys / norm

        acc = np.zeros((h, w), dtype=np.float32)

        scales = np.linspace(
            self.config["scale_min"],
            self.config["scale_max"],
            self.config["vote_scale_steps"]
        )

        for scale in scales:
            for r0 in self.base_radii:
                r = float(r0 * scale * factor)

                if r < 2:
                    continue

                for sign in (-1.0, 1.0):
                    cx = np.round(xs + sign * r * ux).astype(np.int32)
                    cy = np.round(ys + sign * r * uy).astype(np.int32)

                    valid = (
                        (cx >= 0) &
                        (cx < w) &
                        (cy >= 0) &
                        (cy < h)
                    )

                    if np.count_nonzero(valid) == 0:
                        continue

                    flat_idx = cy[valid] * w + cx[valid]

                    vote_weight = mag[valid]
                    vote_weight = np.minimum(
                        vote_weight,
                        self.config["vote_weight_cap"]
                    )

                    acc_flat = np.bincount(
                        flat_idx,
                        weights=vote_weight,
                        minlength=h * w
                    ).astype(np.float32)

                    acc += acc_flat.reshape(h, w)

        acc = cv2.GaussianBlur(
            acc,
            (0, 0),
            sigmaX=self.config["vote_blur_sigma"],
            sigmaY=self.config["vote_blur_sigma"]
        )

        _, max_val, _, max_loc = cv2.minMaxLoc(acc)

        if max_val <= self.config["vote_min_peak"]:
            return None, acc, None

        cx_small, cy_small = max_loc

        center_small = (int(cx_small), int(cy_small))

        center_roi = (
            int(round(cx_small / factor)),
            int(round(cy_small / factor))
        )

        return center_roi, acc, center_small

    # =========================================================
    # scale 估计
    # =========================================================

    def _expected_edge_radii(self, scale, factor=1.0):
        items = []
        linestyles = self.config["linestyles"]
        half = self.config["real_main_thickness"] / 2.0

        for idx, r0 in enumerate(self.base_radii):
            r = float(r0 * scale * factor)
            edge_half = half * factor

            ls = linestyles[idx]
            weight = (
                self.config["solid_weight"]
                if ls == "solid"
                else self.config["dash_weight"]
            )

            items.append((r - edge_half, weight))
            items.append((r + edge_half, weight))

            if ls == "--" and self.config["use_thin_aux_line"]:
                items.append((
                    r + self.config["thin_offset"] * factor,
                    self.config["thin_aux_weight"]
                ))

        return [(r, w) for r, w in items if r > 1]

    def _estimate_scale_by_radial_hist(self, edges, center, factor):
        if center is None:
            return None

        cx, cy = center
        ys, xs = np.where(edges > 0)

        if len(xs) < self.config["scale_min_edge_points"]:
            return None

        dx = xs.astype(np.float32) - cx
        dy = ys.astype(np.float32) - cy
        dist = np.sqrt(dx * dx + dy * dy)

        max_r = int(max(self.base_radii) * self.config["scale_max"] * factor + 40)

        hist, _ = np.histogram(
            dist,
            bins=np.arange(0, max_r + 2, 1)
        )

        hist = hist.astype(np.float32)

        if self.config["scale_hist_smooth"] > 1:
            k = self.config["scale_hist_smooth"]
            kernel = np.ones(k, dtype=np.float32) / k
            hist = np.convolve(hist, kernel, mode="same")

        best_scale = None
        best_score = -1.0

        scales = np.linspace(
            self.config["scale_min"],
            self.config["scale_max"],
            self.config["scale_est_steps"]
        )

        for scale in scales:
            score = 0.0

            for r, weight in self._expected_edge_radii(scale, factor=factor):
                ri = int(round(r))

                if ri < 2 or ri >= len(hist) - 3:
                    continue

                local_peak = np.max(hist[ri - 2:ri + 3])
                score += local_peak * weight

            if score > best_score:
                best_score = float(score)
                best_scale = float(scale)

        return best_scale

    # =========================================================
    # 稀疏模板精修
    # =========================================================

    def _make_sparse_template(self, scale):
        key = round(float(scale), 5)

        if key in self.template_cache:
            return self.template_cache[key]

        items = self._expected_edge_radii(scale, factor=1.0)

        max_r = max(r for r, _ in items)
        half_size = int(math.ceil(max_r + self.config["template_margin"]))
        size = half_size * 2 + 1
        center = (half_size, half_size)

        tmpl = np.zeros((size, size), dtype=np.float32)

        for r, weight in items:
            layer = np.zeros_like(tmpl, dtype=np.float32)

            cv2.circle(
                layer,
                center,
                int(round(r)),
                float(weight),
                thickness=self.config["template_edge_thickness"],
                lineType=cv2.LINE_8
            )

            np.maximum(tmpl, layer, out=tmpl)

        ys, xs = np.where(tmpl > 0)
        weights = tmpl[ys, xs].astype(np.float32)

        dx = xs.astype(np.int32) - center[0]
        dy = ys.astype(np.int32) - center[1]

        offsets = np.stack([dx, dy], axis=1)

        max_points = self.config["max_sparse_points"]

        if len(offsets) > max_points:
            idx = np.linspace(0, len(offsets) - 1, max_points).astype(np.int32)
            offsets = offsets[idx]
            weights = weights[idx]

        data = {
            "offsets": offsets,
            "weights": weights,
        }

        self.template_cache[key] = data
        return data

    def _score_sparse_centers_vectorized(self, dist, centers, scale):
        h, w = dist.shape[:2]

        td = self._make_sparse_template(scale)
        offsets = td["offsets"]
        weights = td["weights"].astype(np.float32)

        centers = np.asarray(centers, dtype=np.int32)

        cx = centers[:, 0:1]
        cy = centers[:, 1:2]

        xs = cx + offsets[:, 0][None, :]
        ys = cy + offsets[:, 1][None, :]

        valid = (
            (xs >= 0) &
            (xs < w) &
            (ys >= 0) &
            (ys < h)
        )

        xs_clip = np.clip(xs, 0, w - 1)
        ys_clip = np.clip(ys, 0, h - 1)

        values = dist[ys_clip, xs_clip].astype(np.float32)

        weight_mat = weights[None, :] * valid.astype(np.float32)

        weight_sum = np.sum(weight_mat, axis=1)
        score = np.sum(values * weight_mat, axis=1) / np.maximum(weight_sum, 1e-6)

        score[weight_sum < self.config["min_valid_sparse_points"]] = np.inf

        return score

    def _refine_local_roi(self, dist_roi, center_roi0, scale0):
        if center_roi0 is None or scale0 is None:
            return None, None, float("inf")

        cx0, cy0 = center_roi0

        centers = []

        for dy in range(
            -self.config["refine_window"],
            self.config["refine_window"] + 1,
            self.config["refine_step"]
        ):
            for dx in range(
                -self.config["refine_window"],
                self.config["refine_window"] + 1,
                self.config["refine_step"]
            ):
                centers.append((cx0 + dx, cy0 + dy))

        best_center = center_roi0
        best_scale = scale0
        best_score = float("inf")

        scale_values = np.linspace(
            scale0 - self.config["refine_scale_range"],
            scale0 + self.config["refine_scale_range"],
            self.config["refine_scale_steps"]
        )

        for scale in scale_values:
            if scale < self.config["scale_min"] or scale > self.config["scale_max"]:
                continue

            scores = self._score_sparse_centers_vectorized(
                dist_roi,
                centers,
                scale
            )

            idx = int(np.argmin(scores))
            score = float(scores[idx])

            if score < best_score:
                best_score = score
                best_center = centers[idx]
                best_scale = float(scale)

        return best_center, best_scale, best_score

    # =========================================================
    # 后验径向验证
    # =========================================================

    def _radial_verify(self, edges, center, scale):
        if center is None or scale is None:
            return 0.0

        cx, cy = center
        ys, xs = np.where(edges > 0)

        if len(xs) < self.config["min_edge_points_for_radial"]:
            return 0.0

        dx = xs.astype(np.float32) - cx
        dy = ys.astype(np.float32) - cy

        dist = np.sqrt(dx * dx + dy * dy)
        angles = np.arctan2(dy, dx)
        angles = (angles + 2 * math.pi) % (2 * math.pi)

        bins = self.config["angle_bins"]
        band = self.config["radial_band"]

        total_score = 0.0
        total_weight = 0.0

        for r, weight in self._expected_edge_radii(scale, factor=1.0):
            mask = np.abs(dist - r) <= band
            count = int(np.count_nonzero(mask))

            if count == 0:
                r_score = 0.0
            else:
                angle_sel = angles[mask]
                bin_ids = np.floor(
                    angle_sel / (2 * math.pi) * bins
                ).astype(np.int32)

                coverage = len(np.unique(bin_ids)) / bins

                expected_points = max(1.0, 2 * math.pi * r * (2 * band + 1))
                density = count / expected_points

                coverage_score = min(
                    coverage / self.config["radial_good_coverage"],
                    1.0
                )
                density_score = min(
                    density * self.config["radial_density_gain"],
                    1.0
                )

                r_score = (
                    self.config["radial_coverage_weight"] * coverage_score
                    + self.config["radial_density_weight"] * density_score
                )

            total_score += r_score * weight
            total_weight += weight

        if total_weight <= 1e-6:
            return 0.0

        return float(total_score / total_weight)

    # =========================================================
    # 结果构造与辅助
    # =========================================================

    def _build_result(self, center, scale, chamfer_score, radial_score, t0):
        if math.isinf(chamfer_score):
            chamfer_conf = 0.0
        else:
            chamfer_conf = max(
                0.0,
                min(
                    1.0,
                    1.0 - (chamfer_score - self.config["chamfer_excellent"]) /
                    (self.config["chamfer_bad"] - self.config["chamfer_excellent"])
                )
            )

        confidence = (
            self.config["confidence_chamfer_weight"] * chamfer_conf
            + self.config["confidence_radial_weight"] * radial_score
        )
        confidence = float(np.clip(confidence, 0.0, 1.0))

        found = (
            center is not None
            and scale is not None
            and confidence >= self.config["min_confidence"]
            and radial_score >= self.config["min_radial_score"]
        )

        time_ms = (time.perf_counter() - t0) * 1000.0

        return CircleDetectResult(
            found=found,
            center=center if found else None,
            scale=scale if found else None,
            confidence=confidence,
            chamfer_score=float(chamfer_score),
            radial_score=float(radial_score),
            time_ms=time_ms
        )

    @staticmethod
    def parse_center_from_filename(image_path):
        path = Path(image_path)
        m = re.search(r"\((\d+),\s*(\d+)\)", path.name)

        if m is None:
            return None

        return int(m.group(1)), int(m.group(2))

    def draw_debug_result(self, raw_img, result, debug=None, gt=None):
        """
        可选调试可视化。
        """
        out = raw_img.copy()

        if debug is not None:
            if debug.get("search_roi_xyxy") is not None:
                x1, y1, x2, y2 = debug["search_roi_xyxy"]
                cv2.rectangle(out, (x1, y1), (x2, y2), (180, 180, 180), 2)

            if debug.get("fine_roi_xyxy") is not None:
                x1, y1, x2, y2 = debug["fine_roi_xyxy"]
                cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 255), 2)

            if debug.get("center0_full") is not None:
                cv2.drawMarker(
                    out,
                    debug["center0_full"],
                    (255, 0, 0),
                    cv2.MARKER_CROSS,
                    35,
                    2
                )

        if result.found and result.center is not None:
            cx, cy = result.center

            cv2.drawMarker(
                out,
                (cx, cy),
                (0, 0, 255),
                cv2.MARKER_CROSS,
                45,
                3
            )
            cv2.circle(out, (cx, cy), 5, (0, 0, 255), -1)

            for r, _ in self._expected_edge_radii(result.scale, factor=1.0):
                cv2.circle(
                    out,
                    (cx, cy),
                    int(round(r)),
                    (0, 255, 255),
                    1,
                    lineType=cv2.LINE_AA
                )

            text = (
                f"center=({cx},{cy}) "
                f"scale={result.scale:.3f} "
                f"conf={result.confidence:.2f} "
                f"time={result.time_ms:.1f}ms"
            )

            if gt is not None:
                err = math.hypot(cx - gt[0], cy - gt[1])
                text += f" err={err:.1f}px"

            cv2.putText(
                out,
                text,
                (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 0, 255),
                2
            )

        if gt is not None:
            cv2.drawMarker(
                out,
                gt,
                (255, 0, 0),
                cv2.MARKER_SQUARE,
                35,
                2
            )

        return out


def main():
    config = {
        # None 表示全图搜索
        "search_roi": None,

        # 如果后续知道目标大概区域，可以改成：
        # "search_roi": (200, 150, 1000, 850),

        "coarse_factor": 0.25,
        "vote_scale_steps": 7,
        "vote_max_edge_points": 7000,
        "fine_roi_margin": 80,
    }

    detector = CircleDetector(config)

    image_path = r"F:\Project\littleCar2\zhengdian\4-29\jetson\scripts\circle\(567, 685).jpg"

    result = detector.detect(
        image_path=image_path,
        return_debug=False
    )

    if result.found and result.center is not None:
        print(result.center)
    else:
        print(None)


if __name__ == "__main__":
    main()


