import cv2
import re
import csv
import math
import time
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from dataclasses import dataclass
from typing import Optional, Tuple, Dict, Any


plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "sans-serif"]
plt.rcParams["axes.unicode_minus"] = False


@dataclass
class MatchResult:
    found: bool
    center: Optional[Tuple[int, int]]
    scale: Optional[float]
    chamfer_score: float
    radial_score: float
    confidence: float
    error_px: Optional[float] = None
    time_ms: float = 0.0


class RadialVoteConcentricMatcher:
    def __init__(self, cfg: Dict[str, Any]):
        self.cfg = cfg
        self.base_radii = np.array(
            cfg["base_radii"],
            dtype=np.float32
        ) * cfg["base_radius_gain"]

        self.template_cache = {}

    def parse_gt_from_filename(self, image_path):
        path = Path(image_path)
        m = re.search(r"\((\d+),\s*(\d+)\)", path.name)

        if m is None:
            return None

        return int(m.group(1)), int(m.group(2))

    def clip_roi_xyxy(self, img, roi):
        h, w = img.shape[:2]

        if roi is None:
            return img, (0, 0), (0, 0, w, h)

        x1, y1, x2, y2 = roi

        x1 = max(0, min(w - 1, int(x1)))
        y1 = max(0, min(h - 1, int(y1)))
        x2 = max(x1 + 1, min(w, int(x2)))
        y2 = max(y1 + 1, min(h, int(y2)))

        return img[y1:y2, x1:x2].copy(), (x1, y1), (x1, y1, x2, y2)

    def preprocess_edges(self, img, factor=1.0, for_vote=False):
        """
        输出 gray_norm, edges。
        factor < 1 时先缩放图像，用于粗定位。
        """
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
            clipLimit=self.cfg["clahe_limit"],
            tileGridSize=self.cfg["clahe_grid"]
        )
        gray_clahe = clahe.apply(gray)

        bg_sigma = max(3.0, self.cfg["bg_sigma"] * factor)
        bg = cv2.GaussianBlur(
            gray_clahe,
            (0, 0),
            sigmaX=bg_sigma,
            sigmaY=bg_sigma
        )

        gray_norm = cv2.addWeighted(
            gray_clahe,
            self.cfg["norm_alpha"],
            bg,
            self.cfg["norm_beta"],
            self.cfg["norm_gamma"]
        )
        gray_norm = np.clip(gray_norm, 0, 255).astype(np.uint8)

        if for_vote and self.cfg["vote_fast_preprocess"]:
            gray_denoise = cv2.GaussianBlur(gray_norm, (5, 5), 1.0)
        else:
            if self.cfg["denoise_method"] == "bilateral":
                gray_denoise = cv2.bilateralFilter(
                    gray_norm,
                    d=self.cfg["bilateral_d"],
                    sigmaColor=self.cfg["bilateral_sigma_color"],
                    sigmaSpace=self.cfg["bilateral_sigma_space"]
                )
            else:
                gray_denoise = cv2.GaussianBlur(
                    gray_norm,
                    self.cfg["gaussian_ksize"],
                    self.cfg["gaussian_sigma"]
                )

        edges = cv2.Canny(
            gray_denoise,
            self.cfg["canny_low"],
            self.cfg["canny_high"]
        )

        if self.cfg["remove_long_lines"] and not for_vote:
            edges, _ = self.remove_long_lines(edges, factor=factor)

        if self.cfg["remove_small_components"] and not for_vote:
            edges = self.remove_small_components(edges, factor=factor)

        return gray_norm, gray_denoise, edges

    def remove_long_lines(self, edges, factor=1.0):
        line_len = max(20, int(self.cfg["line_len"] * factor))
        line_gap = max(5, int(self.cfg["line_gap"] * factor))
        line_thr = max(20, int(self.cfg["line_thr"] * factor))
        erase_thick = max(1, int(self.cfg["line_erase_thickness"] * factor))

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

    def remove_small_components(self, edges, factor=1.0):
        clean = edges.copy()
        min_area = max(3, int(self.cfg["min_edge_area"] * factor * factor))

        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
            clean,
            connectivity=8
        )

        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]

            if area < min_area:
                clean[labels == i] = 0

        return clean

    def distance_map_from_edges(self, edges):
        inv_edges = 255 - edges

        dist = cv2.distanceTransform(
            inv_edges,
            cv2.DIST_L2,
            self.cfg["distance_mask_size"]
        )

        dist = np.minimum(dist, self.cfg["distance_cap"])
        return dist.astype(np.float32)

    def radial_vote_center(self, gray, edges, factor):
        """
        径向投票粗定位圆心。

        输入是 search_roi 缩放后的灰度和边缘。
        返回的是 search_roi 原图坐标系下的中心，不含全图偏移。
        """
        h, w = edges.shape[:2]

        gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
        gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)

        ys, xs = np.where(edges > 0)

        if len(xs) < self.cfg["vote_min_edge_points"]:
            return None, None

        gxs = gx[ys, xs]
        gys = gy[ys, xs]
        mag = np.sqrt(gxs * gxs + gys * gys)

        valid_grad = mag > self.cfg["vote_min_gradient"]
        xs = xs[valid_grad]
        ys = ys[valid_grad]
        gxs = gxs[valid_grad]
        gys = gys[valid_grad]
        mag = mag[valid_grad]

        if len(xs) < self.cfg["vote_min_edge_points"]:
            return None, None

        max_points = self.cfg["vote_max_edge_points"]

        if len(xs) > max_points:
            # 取梯度强度较大的边缘点，速度更快，投票更稳
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
            self.cfg["scale_min"],
            self.cfg["scale_max"],
            self.cfg["vote_scale_steps"]
        )

        # 粗投票只用主半径，不用内外边缘半径，降低计算量
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
                    vote_weight = np.minimum(vote_weight, self.cfg["vote_weight_cap"])

                    acc_flat = np.bincount(
                        flat_idx,
                        weights=vote_weight,
                        minlength=h * w
                    ).astype(np.float32)

                    acc += acc_flat.reshape(h, w)

        acc = cv2.GaussianBlur(
            acc,
            (0, 0),
            sigmaX=self.cfg["vote_blur_sigma"],
            sigmaY=self.cfg["vote_blur_sigma"]
        )

        _, max_val, _, max_loc = cv2.minMaxLoc(acc)

        if max_val <= self.cfg["vote_min_peak"]:
            return None, acc

        cx_small, cy_small = max_loc

        cx_roi = int(round(cx_small / factor))
        cy_roi = int(round(cy_small / factor))

        return (cx_roi, cy_roi), acc

    def expected_edge_radii(self, scale):
        """
        生成精修模板使用的边缘半径。
        由于 Canny 会检测粗线的内外边缘，所以每个主圆给出 r±thickness/2。
        """
        items = []
        linestyles = self.cfg["linestyles"]
        half = self.cfg["real_main_thickness"] / 2.0

        for idx, r0 in enumerate(self.base_radii):
            r = float(r0 * scale)
            ls = linestyles[idx]

            weight = self.cfg["solid_weight"] if ls == "solid" else self.cfg["dash_weight"]

            items.append((r - half, weight))
            items.append((r + half, weight))

            if ls == "--" and self.cfg["use_thin_aux_line"]:
                items.append((
                    r + self.cfg["thin_offset"],
                    self.cfg["thin_aux_weight"]
                ))

        return [(r, w) for r, w in items if r > 1]

    def estimate_scale_by_radial_hist(self, edges, center_roi):
        """
        根据粗圆心估计 scale。
        在 search_roi 坐标系中计算。
        """
        if center_roi is None:
            return None

        cx, cy = center_roi
        ys, xs = np.where(edges > 0)

        if len(xs) < self.cfg["scale_min_edge_points"]:
            return None

        dx = xs.astype(np.float32) - cx
        dy = ys.astype(np.float32) - cy
        dist = np.sqrt(dx * dx + dy * dy)

        max_r = int(max(self.base_radii) * self.cfg["scale_max"] + 40)
        hist, _ = np.histogram(
            dist,
            bins=np.arange(0, max_r + 2, 1)
        )

        hist = hist.astype(np.float32)

        if self.cfg["scale_hist_smooth"] > 1:
            k = self.cfg["scale_hist_smooth"]
            kernel = np.ones(k, dtype=np.float32) / k
            hist = np.convolve(hist, kernel, mode="same")

        best_scale = None
        best_score = -1.0

        scales = np.linspace(
            self.cfg["scale_min"],
            self.cfg["scale_max"],
            self.cfg["scale_est_steps"]
        )

        for scale in scales:
            score = 0.0

            for r, weight in self.expected_edge_radii(scale):
                ri = int(round(r))

                if ri < 2 or ri >= len(hist) - 3:
                    continue

                local_peak = np.max(hist[ri - 2:ri + 3])
                score += local_peak * weight

            if score > best_score:
                best_score = float(score)
                best_scale = float(scale)

        return best_scale

    def make_sparse_template(self, scale):
        """
        稀疏模板点，用于 ROI 内 Chamfer 精修。
        """
        key = round(float(scale), 5)

        if key in self.template_cache:
            return self.template_cache[key]

        items = self.expected_edge_radii(scale)

        max_r = max(r for r, _ in items)
        half_size = int(math.ceil(max_r + self.cfg["template_margin"]))
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
                thickness=self.cfg["template_edge_thickness"],
                lineType=cv2.LINE_8
            )

            np.maximum(tmpl, layer, out=tmpl)

        ys, xs = np.where(tmpl > 0)
        weights = tmpl[ys, xs].astype(np.float32)

        dx = xs.astype(np.int32) - center[0]
        dy = ys.astype(np.int32) - center[1]

        offsets = np.stack([dx, dy], axis=1)

        max_points = self.cfg["max_sparse_points"]

        if len(offsets) > max_points:
            idx = np.linspace(0, len(offsets) - 1, max_points).astype(np.int32)
            offsets = offsets[idx]
            weights = weights[idx]

        tmpl_vis = np.clip(tmpl * 255, 0, 255).astype(np.uint8)

        data = {
            "offsets": offsets,
            "weights": weights,
            "template_vis": tmpl_vis
        }

        self.template_cache[key] = data
        return data

    def crop_fine_roi(self, raw_img, center_full, scale):
        h, w = raw_img.shape[:2]
        cx, cy = center_full

        max_r = max(self.base_radii) * scale
        half = int(max_r + self.cfg["fine_roi_margin"])

        x1 = max(0, cx - half)
        y1 = max(0, cy - half)
        x2 = min(w, cx + half)
        y2 = min(h, cy + half)

        roi = raw_img[y1:y2, x1:x2].copy()
        return roi, (x1, y1), (x1, y1, x2, y2)

    def score_sparse_centers_vectorized(self, dist, centers, scale):
        """
        对一批中心点同时评分，避免 Python 三重循环过慢。
        """
        h, w = dist.shape[:2]

        td = self.make_sparse_template(scale)
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

        score[weight_sum < self.cfg["min_valid_sparse_points"]] = np.inf

        return score

    def refine_local_roi(self, dist_roi, center_roi0, scale0):
        """
        在 fine ROI 内精修圆心和 scale。
        """
        if center_roi0 is None or scale0 is None:
            return None, None, float("inf")

        cx0, cy0 = center_roi0

        dx_values = range(
            -self.cfg["refine_window"],
            self.cfg["refine_window"] + 1,
            self.cfg["refine_step"]
        )
        dy_values = range(
            -self.cfg["refine_window"],
            self.cfg["refine_window"] + 1,
            self.cfg["refine_step"]
        )

        centers = []
        for dy in dy_values:
            for dx in dx_values:
                centers.append((cx0 + dx, cy0 + dy))

        best_center = center_roi0
        best_scale = scale0
        best_score = float("inf")

        scale_values = np.linspace(
            scale0 - self.cfg["refine_scale_range"],
            scale0 + self.cfg["refine_scale_range"],
            self.cfg["refine_scale_steps"]
        )

        for scale in scale_values:
            if scale < self.cfg["scale_min"] or scale > self.cfg["scale_max"]:
                continue

            scores = self.score_sparse_centers_vectorized(
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

    def radial_verify(self, edges, center, scale):
        """
        后验验证：预期半径附近的边缘覆盖率和密度。
        """
        if center is None or scale is None:
            return 0.0

        cx, cy = center
        ys, xs = np.where(edges > 0)

        if len(xs) < self.cfg["min_edge_points_for_radial"]:
            return 0.0

        dx = xs.astype(np.float32) - cx
        dy = ys.astype(np.float32) - cy

        dist = np.sqrt(dx * dx + dy * dy)
        angles = np.arctan2(dy, dx)
        angles = (angles + 2 * math.pi) % (2 * math.pi)

        bins = self.cfg["angle_bins"]
        band = self.cfg["radial_band"]

        total_score = 0.0
        total_weight = 0.0

        for r, weight in self.expected_edge_radii(scale):
            mask = np.abs(dist - r) <= band
            count = int(np.count_nonzero(mask))

            if count == 0:
                r_score = 0.0
            else:
                angle_sel = angles[mask]
                bin_ids = np.floor(angle_sel / (2 * math.pi) * bins).astype(np.int32)

                coverage = len(np.unique(bin_ids)) / bins

                expected_points = max(1.0, 2 * math.pi * r * (2 * band + 1))
                density = count / expected_points

                coverage_score = min(
                    coverage / self.cfg["radial_good_coverage"],
                    1.0
                )
                density_score = min(
                    density * self.cfg["radial_density_gain"],
                    1.0
                )

                r_score = (
                    self.cfg["radial_coverage_weight"] * coverage_score
                    + self.cfg["radial_density_weight"] * density_score
                )

            total_score += r_score * weight
            total_weight += weight

        if total_weight <= 1e-6:
            return 0.0

        return float(total_score / total_weight)

    def build_result(self, center, scale, chamfer_score, radial_score, t0):
        chamfer_conf = max(
            0.0,
            min(
                1.0,
                1.0 - (chamfer_score - self.cfg["chamfer_excellent"]) /
                (self.cfg["chamfer_bad"] - self.cfg["chamfer_excellent"])
            )
        )

        confidence = (
            self.cfg["confidence_chamfer_weight"] * chamfer_conf
            + self.cfg["confidence_radial_weight"] * radial_score
        )
        confidence = float(np.clip(confidence, 0.0, 1.0))

        found = (
            center is not None
            and scale is not None
            and confidence >= self.cfg["min_confidence"]
            and radial_score >= self.cfg["min_radial_score"]
        )

        time_ms = (time.perf_counter() - t0) * 1000.0

        return MatchResult(
            found=found,
            center=center if found else None,
            scale=scale if found else None,
            chamfer_score=float(chamfer_score),
            radial_score=float(radial_score),
            confidence=confidence,
            time_ms=time_ms
        )

    def locate(self, raw_img):
        t0 = time.perf_counter()

        # 1. search ROI，默认全图
        search_img, search_origin, search_roi_xyxy = self.clip_roi_xyxy(
            raw_img,
            self.cfg["search_roi"]
        )

        # 2. 粗定位：只在 search ROI 的低分辨率图上做径向投票
        factor = self.cfg["coarse_factor"]
        gray_small, denoise_small, edges_small = self.preprocess_edges(
            search_img,
            factor=factor,
            for_vote=True
        )

        center_search_roi, vote_acc = self.radial_vote_center(
            gray_small,
            edges_small,
            factor=factor
        )

        if center_search_roi is None:
            result = self.build_result(
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
                "center_search_roi": None
            }
            return result, debug

        # 3. 在 search ROI 中估计 scale
        scale0 = self.estimate_scale_by_radial_hist(
            edges_small,
            (
                int(round(center_search_roi[0] * factor)),
                int(round(center_search_roi[1] * factor))
            )
        )

        if scale0 is None:
            scale0 = (self.cfg["scale_min"] + self.cfg["scale_max"]) / 2.0

        # 4. 映射到全图坐标
        center0_full = (
            center_search_roi[0] + search_origin[0],
            center_search_roi[1] + search_origin[1]
        )

        # 5. 裁剪 fine ROI
        fine_roi, fine_origin, fine_roi_xyxy = self.crop_fine_roi(
            raw_img,
            center0_full,
            scale0
        )

        # 6. 只对 fine ROI 做原图级边缘和距离变换
        gray_roi, denoise_roi, edges_roi = self.preprocess_edges(
            fine_roi,
            factor=1.0,
            for_vote=False
        )
        dist_roi = self.distance_map_from_edges(edges_roi)

        center_roi0 = (
            center0_full[0] - fine_origin[0],
            center0_full[1] - fine_origin[1]
        )

        # 7. fine ROI 内稀疏模板精修
        center_roi_refined, scale_refined, chamfer_score = self.refine_local_roi(
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

            radial_score = self.radial_verify(
                edges_roi,
                center_roi_refined,
                scale_refined
            )

        result = self.build_result(
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
            "radial_score": radial_score
        }

        return result, debug

    def draw_result(self, raw_img, result, debug, gt=None):
        out = raw_img.copy()

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

            for r, _ in self.expected_edge_radii(result.scale):
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
        else:
            cv2.putText(
                out,
                f"not found conf={result.confidence:.2f} time={result.time_ms:.1f}ms",
                (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
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

    def show_pipeline(self, raw_img, result, debug, gt=None):
        final_img = self.draw_result(raw_img, result, debug, gt=gt)

        if debug.get("vote_acc") is not None:
            vote_vis = cv2.normalize(
                debug["vote_acc"],
                None,
                0,
                255,
                cv2.NORM_MINMAX
            ).astype(np.uint8)
        else:
            vote_vis = np.zeros_like(debug["edges_small"])

        if debug.get("dist_roi") is not None:
            dist_vis = cv2.normalize(
                debug["dist_roi"],
                None,
                0,
                255,
                cv2.NORM_MINMAX
            ).astype(np.uint8)
        else:
            dist_vis = np.zeros_like(debug["edges_small"])

        plot_list = [
            (debug["search_img"], "1. search ROI，默认全图"),
            (debug["edges_small"], "2. 低分辨率边缘"),
            (vote_vis, "3. 径向投票累加图"),
            (debug.get("fine_roi", np.zeros_like(raw_img)), "4. fine ROI"),
            (debug.get("edges_roi", np.zeros_like(debug["edges_small"])), "5. fine ROI 边缘"),
            (dist_vis, "6. fine ROI 距离图"),
            (final_img, "7. 最终定位结果"),
        ]

        plt.figure(figsize=(22, 12))

        for idx, (img, title) in enumerate(plot_list):
            plt.subplot(2, 4, idx + 1)

            if len(img.shape) == 2:
                plt.imshow(img, cmap="gray")
            else:
                plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))

            plt.title(title, fontsize=12)
            plt.axis("off")

        plt.tight_layout()
        plt.show()


def test_single_image(image_path, matcher, show=True, save_debug=True):
    image_path = Path(image_path)
    raw_img = cv2.imread(str(image_path))

    if raw_img is None:
        raise RuntimeError(f"无法读取图像：{image_path}")

    gt = matcher.parse_gt_from_filename(image_path)

    result, debug = matcher.locate(raw_img)

    err = None
    if result.found and result.center is not None and gt is not None:
        err = math.hypot(result.center[0] - gt[0], result.center[1] - gt[1])
        result.error_px = err

    print("单张检测结果")
    print(f"image: {image_path.name}")
    print(f"found: {result.found}")
    print(f"center: {result.center}")
    print(f"scale: {result.scale}")
    print(f"chamfer_score: {result.chamfer_score:.3f}")
    print(f"radial_score: {result.radial_score:.3f}")
    print(f"confidence: {result.confidence:.3f}")
    print(f"time_ms: {result.time_ms:.2f}")

    if gt is not None:
        print(f"gt: {gt}")
        print(f"error_px: {err:.3f}" if err is not None else "error_px: None")

    if save_debug:
        debug_dir = image_path.parent / "debug_radial_vote"
        debug_dir.mkdir(exist_ok=True)

        final_img = matcher.draw_result(raw_img, result, debug, gt=gt)
        cv2.imwrite(str(debug_dir / f"{image_path.stem}_final.png"), final_img)

        if debug.get("edges_roi") is not None:
            cv2.imwrite(str(debug_dir / f"{image_path.stem}_roi_edges.png"), debug["edges_roi"])

    if show:
        matcher.show_pipeline(raw_img, result, debug, gt=gt)

    return result


def test_folder(image_dir, matcher, save_debug=False):
    image_dir = Path(image_dir)

    image_paths = []
    for ext in ["*.png", "*.jpg", "*.jpeg", "*.bmp"]:
        image_paths.extend(image_dir.glob(ext))

    image_paths = sorted(image_paths)

    if len(image_paths) == 0:
        raise RuntimeError(f"目录中没有图像：{image_dir}")

    errors = []
    times = []
    found_count = 0
    records = []

    debug_dir = image_dir / "debug_radial_vote"
    if save_debug:
        debug_dir.mkdir(exist_ok=True)

    for image_path in image_paths:
        raw_img = cv2.imread(str(image_path))
        if raw_img is None:
            continue

        gt = matcher.parse_gt_from_filename(image_path)
        result, debug = matcher.locate(raw_img)

        times.append(result.time_ms)

        err = None
        if result.found:
            found_count += 1

        if result.found and result.center is not None and gt is not None:
            err = math.hypot(result.center[0] - gt[0], result.center[1] - gt[1])
            result.error_px = err
            errors.append(err)

        records.append([
            image_path.name,
            result.found,
            result.center[0] if result.center else None,
            result.center[1] if result.center else None,
            result.scale,
            result.chamfer_score,
            result.radial_score,
            result.confidence,
            gt[0] if gt else None,
            gt[1] if gt else None,
            err,
            result.time_ms
        ])

        if save_debug:
            final_img = matcher.draw_result(raw_img, result, debug, gt=gt)
            cv2.imwrite(str(debug_dir / f"{image_path.stem}_final.png"), final_img)

    csv_path = image_dir / "radial_vote_results.csv"

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "filename",
            "found",
            "pred_cx",
            "pred_cy",
            "scale",
            "chamfer_score",
            "radial_score",
            "confidence",
            "gt_cx",
            "gt_cy",
            "error_px",
            "time_ms"
        ])
        writer.writerows(records)

    print("\n批量检测统计")
    print(f"图像数量: {len(image_paths)}")
    print(f"成功检测: {found_count}")
    print(f"成功率: {found_count / len(image_paths) * 100:.2f}%")
    print(f"结果文件: {csv_path}")

    if len(errors) > 0:
        errors = np.array(errors, dtype=np.float32)
        print(f"平均误差: {errors.mean():.3f}px")
        print(f"中位误差: {np.median(errors):.3f}px")
        print(f"最大误差: {errors.max():.3f}px")
        print(f"95%误差: {np.percentile(errors, 95):.3f}px")

    if len(times) > 0:
        times = np.array(times, dtype=np.float32)
        print(f"平均耗时: {times.mean():.2f}ms")
        print(f"中位耗时: {np.median(times):.2f}ms")
        print(f"最大耗时: {times.max():.2f}ms")

    return records


def main():
    CONFIG = {
        # =========================
        # 运行模式
        # =========================
        "mode": "single",  # "single" 或 "folder"

        "image_path": r"F:\Project\littleCar2\zhengdian\4-29\jetson\scripts\circle\(567, 685).jpg",
        "image_dir": r"F:\Project\littleCar2\zhengdian\4-29\jetson\scripts\circle",

        "show": True,
        "save_debug": True,

        # =========================
        # 搜索 ROI
        # =========================
        # None 表示全图搜索
        # 如果你知道目标大概区域，可以设为 (x1, y1, x2, y2)
        # 例如："search_roi": (200, 150, 1000, 850)
        "search_roi": None,

        # 径向投票后，原图精修 ROI 的额外边距
        "fine_roi_margin": 80,

        # =========================
        # 图案结构
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

        # 精修 ROI 默认用 gaussian 更快
        # 如果噪声明显，可改成 bilateral
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
        # =========================
        # 默认关闭，速度更快。模板匹配本身对少量干扰线不敏感。
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
    }

    matcher = RadialVoteConcentricMatcher(CONFIG)

    if CONFIG["mode"] == "single":
        test_single_image(
            CONFIG["image_path"],
            matcher,
            show=CONFIG["show"],
            save_debug=CONFIG["save_debug"]
        )

    elif CONFIG["mode"] == "folder":
        test_folder(
            CONFIG["image_dir"],
            matcher,
            save_debug=CONFIG["save_debug"]
        )

    else:
        raise ValueError("CONFIG['mode'] 只能是 'single' 或 'folder'")


if __name__ == "__main__":
    main()