"""同心圆标志定位和数字识别函数。"""

from __future__ import annotations

import itertools
import math
from functools import lru_cache
from typing import Any

import cv2
import numpy as np
from scipy.optimize import least_squares


_NOMINAL_RADII = np.array([28.0, 30.5, 35.5, 40.2, 44.9, 49.6], dtype=np.float64)
_MIN_CONFIDENCE = 0.45


def detect_numbered_marker(frame_bgr: np.ndarray) -> dict[str, Any] | None:
    """从一帧 BGR 图像中定位同心圆标志并识别中心数字。"""
    if not isinstance(frame_bgr, np.ndarray) or frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
        raise TypeError("frame_bgr must be a BGR numpy.ndarray")

    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    located = _locate_concentric_marker(gray)
    if located is None:
        return None

    center_x, center_y, scale_factor, rings, confidence = located
    digit, digit_angle_deg, digit_confidence = _classify_center_digit(gray, center_x, center_y, scale_factor)
    return {
        "center_x": center_x,
        "center_y": center_y,
        "digit": digit,
        "digit_angle_deg": digit_angle_deg,
        "confidence": min(confidence, digit_confidence) if digit is not None else confidence,
        "ring_count": len(rings),
        "scale_factor": scale_factor,
    }


def _locate_concentric_marker(gray: np.ndarray):
    height, width = gray.shape[:2]
    minimum_dimension = min(height, width)
    block_size = max(3, int(minimum_dimension * 0.16) | 1)
    threshold = cv2.adaptiveThreshold(
        gray,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        block_size,
        15,
    )
    candidates = _find_ellipse_candidates(threshold)
    cluster = _select_concentric_cluster(candidates, minimum_dimension)
    if len(cluster) < 2:
        return None

    rings = _fit_verified_rings(cluster, minimum_dimension)
    if len(rings) < 2:
        return None

    ring_indices, scale_factor = _match_rings(rings)
    rough_center = tuple(map(float, rings[0]["model"][0]))
    center_x, center_y = _correct_perspective_center(
        [ring["inliers"] for ring in rings],
        ring_indices,
        rough_center,
        scale_factor,
    )
    confidence = _score_marker(rings, minimum_dimension)
    if confidence < _MIN_CONFIDENCE:
        return None
    return center_x, center_y, scale_factor, rings, confidence


def _find_ellipse_candidates(threshold: np.ndarray) -> list[dict[str, Any]]:
    height, width = threshold.shape[:2]
    minimum_dimension = min(height, width)
    minimum_area = height * width * 0.0012
    contours, _ = cv2.findContours(threshold, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    candidates: list[dict[str, Any]] = []
    for contour in contours:
        if cv2.contourArea(contour) < minimum_area or len(contour) < 5:
            continue
        _, _, box_width, box_height = cv2.boundingRect(contour)
        if not 0.55 < box_width / max(box_height, 1) < 1.85:
            continue
        points = contour.reshape(-1, 2).astype(np.float32)
        model = _fit_ellipse(points)
        if _is_valid_ellipse(model, minimum_dimension):
            candidates.append({"model": model, "points": points})
    return candidates


def _fit_ellipse(points: np.ndarray):
    if len(points) < 5:
        return None
    try:
        return cv2.fitEllipse(points.astype(np.float32))
    except cv2.error:
        return None


def _is_valid_ellipse(model: Any, minimum_dimension: int) -> bool:
    if model is None:
        return False
    (center_x, center_y), (axis_a, axis_b), _ = model
    if not np.isfinite([center_x, center_y, axis_a, axis_b]).all() or min(axis_a, axis_b) <= 0:
        return False
    if max(axis_a, axis_b) > minimum_dimension * 0.95 or min(axis_a, axis_b) < 10.0:
        return False
    return min(axis_a, axis_b) / max(axis_a, axis_b) >= 0.50


def _select_concentric_cluster(candidates: list[dict[str, Any]], minimum_dimension: int) -> list[dict[str, Any]]:
    tolerance = minimum_dimension * 0.05
    clusters: list[list[dict[str, Any]]] = []
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

    ordered = sorted(max(clusters, key=len), key=lambda item: max(item["model"][1]))
    gap = minimum_dimension * 0.025
    selected: list[dict[str, Any]] = []
    for candidate in ordered:
        if not selected or max(candidate["model"][1]) - max(selected[-1]["model"][1]) > gap:
            selected.append(candidate)
    return selected[-3:]


def _fit_verified_rings(cluster: list[dict[str, Any]], minimum_dimension: int) -> list[dict[str, Any]]:
    threshold = max(1.0, minimum_dimension * 0.00375)
    verified: list[dict[str, Any]] = []
    for index, candidate in enumerate(cluster):
        model, inliers = _ransac_ellipse_fit(candidate["points"], minimum_dimension, threshold, index)
        if model is not None and len(inliers) >= 5:
            verified.append({"model": model, "inliers": candidate["points"][inliers]})
    return verified


def _ransac_ellipse_fit(points: np.ndarray, minimum_dimension: int, threshold: float, seed_offset: int):
    if len(points) < 5:
        return None, np.empty(0, dtype=np.int32)
    evaluation_points = points[::max(1, len(points) // 150)]
    if len(evaluation_points) < 5:
        return None, np.empty(0, dtype=np.int32)

    rng = np.random.default_rng(20260721 + seed_offset)
    best_model = None
    best_count = 0
    for _ in range(30):
        model = _fit_ellipse(evaluation_points[rng.choice(len(evaluation_points), 5, replace=False)])
        if not _is_valid_ellipse(model, minimum_dimension):
            continue
        count = int(np.count_nonzero(_ellipse_residuals(evaluation_points, model) < threshold))
        if count > best_count:
            best_model, best_count = model, count
    if best_model is None:
        return None, np.empty(0, dtype=np.int32)

    inliers = np.flatnonzero(_ellipse_residuals(points, best_model) < threshold)
    return (_fit_ellipse(points[inliers]) if len(inliers) >= 5 else best_model), inliers


def _ellipse_residuals(points: np.ndarray, model: Any) -> np.ndarray:
    (center_x, center_y), (width, height), angle = model
    radians = np.deg2rad(angle)
    cosine, sine = np.cos(radians), np.sin(radians)
    delta_x, delta_y = points[:, 0] - center_x, points[:, 1] - center_y
    local_x = delta_x * cosine + delta_y * sine
    local_y = -delta_x * sine + delta_y * cosine
    semi_x, semi_y = max(width / 2.0, 1e-6), max(height / 2.0, 1e-6)
    normalized_radius = np.hypot(local_x / semi_x, local_y / semi_y)
    return np.hypot(local_x, local_y) * np.abs(1.0 - 1.0 / np.maximum(normalized_radius, 1e-6))


def _match_rings(rings: list[dict[str, Any]]) -> tuple[list[int], float]:
    measured = np.array([max(ring["model"][1]) / 2.0 for ring in rings], dtype=np.float64)
    best_indices: list[int] = []
    best_scale = 1.0
    best_error = float("inf")
    for indices in itertools.combinations(range(len(_NOMINAL_RADII)), len(measured)):
        nominal = _NOMINAL_RADII[list(indices)]
        scale = float(np.mean(nominal / measured))
        error = float(np.sum(((scale * measured - nominal) / nominal) ** 2))
        if error < best_error:
            best_indices, best_scale, best_error = list(indices), scale, error
    return best_indices, best_scale


def _correct_perspective_center(points_list, ring_indices: list[int], rough_center, scale: float) -> tuple[float, float]:
    center_x, center_y = rough_center
    initial = np.array([scale, 0.0, -scale * center_x, 0.0, scale, -scale * center_y, 0.0, 0.0])
    sampled = [points[::max(1, len(points) // 10)] for points in points_list]
    try:
        result = least_squares(_homography_residual, initial, args=(sampled, ring_indices), method="lm")
        parameters = result.x
        homography = np.array(
            [[parameters[0], parameters[1], parameters[2]], [parameters[3], parameters[4], parameters[5]], [parameters[6], parameters[7], 1.0]]
        )
        center = np.linalg.inv(homography) @ np.array([0.0, 0.0, 1.0])
        return float(center[0] / center[2]), float(center[1] / center[2])
    except (ValueError, np.linalg.LinAlgError):
        return float(center_x), float(center_y)


def _homography_residual(parameters: np.ndarray, points_list, ring_indices: list[int]) -> np.ndarray:
    homography = np.array(
        [[parameters[0], parameters[1], parameters[2]], [parameters[3], parameters[4], parameters[5]], [parameters[6], parameters[7], 1.0]]
    )
    residuals: list[float] = []
    for points, index in zip(points_list, ring_indices):
        projected = np.column_stack((points, np.ones(len(points)))) @ homography.T
        denominator = np.where(np.abs(projected[:, 2]) < 1e-6, 1e-6, projected[:, 2])
        residuals.extend(np.hypot(projected[:, 0] / denominator, projected[:, 1] / denominator) - _NOMINAL_RADII[index])
    return np.asarray(residuals)


def _score_marker(rings: list[dict[str, Any]], minimum_dimension: int) -> float:
    residuals = [np.mean(_ellipse_residuals(ring["inliers"], ring["model"])) for ring in rings]
    chamfer_score = float(np.mean(residuals))
    centers = np.array([ring["model"][0] for ring in rings], dtype=np.float64)
    spread = float(np.mean(np.linalg.norm(centers - centers.mean(axis=0), axis=1)))
    radial_score = float(np.clip(1.0 - spread / (minimum_dimension * 0.05), 0.0, 1.0))
    ring_score = min(1.0, len(rings) / 3.0)
    residual_score = float(np.clip(1.0 - chamfer_score / max(1.0, minimum_dimension * 0.00375), 0.0, 1.0))
    return float(np.clip(0.45 * residual_score + 0.35 * radial_score + 0.20 * ring_score, 0.0, 1.0))


def _classify_center_digit(gray: np.ndarray, center_x: float, center_y: float, scale_factor: float):
    half_size = max(8, int((26.5 / scale_factor) * 0.82))
    x1, x2 = max(0, int(center_x - half_size)), min(gray.shape[1], int(center_x + half_size))
    y1, y2 = max(0, int(center_y - half_size)), min(gray.shape[0], int(center_y + half_size))
    roi = gray[y1:y2, x1:x2]
    if roi.size == 0:
        return None, None, 0.0
    _, binary = cv2.threshold(roi, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    standard = _standardize_digit(binary)
    best_digit: str | None = None
    best_angle: int | None = None
    best_score = -1.0
    for digit, angles in _digit_templates().items():
        for angle, template in angles.items():
            union = np.logical_or(standard > 0, template > 0).sum()
            score = float(np.logical_and(standard > 0, template > 0).sum() / union) if union else 0.0
            if score > best_score:
                best_digit, best_angle, best_score = digit, angle, score
    return best_digit, best_angle, max(0.0, best_score)


def _standardize_digit(binary: np.ndarray) -> np.ndarray:
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    canvas = np.zeros((64, 64), dtype=np.uint8)
    if not contours:
        return canvas
    x, y, width, height = cv2.boundingRect(max(contours, key=cv2.contourArea))
    maximum = max(width, height)
    if maximum == 0:
        return canvas
    resized = cv2.resize(binary[y:y + height, x:x + width], (max(1, int(width * 46.0 / maximum)), max(1, int(height * 46.0 / maximum))), interpolation=cv2.INTER_NEAREST)
    offset_x, offset_y = (64 - resized.shape[1]) // 2, (64 - resized.shape[0]) // 2
    canvas[offset_y:offset_y + resized.shape[0], offset_x:offset_x + resized.shape[1]] = resized
    return canvas


@lru_cache(maxsize=1)
def _digit_templates() -> dict[str, dict[int, np.ndarray]]:
    templates: dict[str, dict[int, np.ndarray]] = {}
    for digit in ("1", "2", "3"):
        source = np.zeros((128, 128), dtype=np.uint8)
        cv2.putText(source, digit, (30, 95), cv2.FONT_HERSHEY_SIMPLEX, 3.2, 255, 9, cv2.LINE_AA)
        center = (source.shape[1] // 2, source.shape[0] // 2)
        templates[digit] = {
            angle: _standardize_digit(cv2.warpAffine(source, cv2.getRotationMatrix2D(center, angle, 1.0), (128, 128), flags=cv2.INTER_NEAREST))
            for angle in range(-60, 61, 5)
        }
    return templates
