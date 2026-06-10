#!/usr/bin/env python3
"""Single-scan leg classifier: Leigh et al. ICRA 2015 (leg_tracker), BSD-3.

Numpy port of cluster_features.cpp's 17 geometric features plus the
pretrained OpenCV random forest (trained_leg_detector_res=0.33.yaml from
github.com/angusleigh/leg_tracker). score() returns P(cluster is a leg)
for one jump-distance cluster of a 2D lidar scan, from a single scan --
no motion needed. Feature definitions intentionally replicate the C++
quirks (iav/ang_diff divided by num_points, radius hardcoded 0) because
the forest was trained on exactly those.
"""
import numpy as np

try:
    import cv2
except ImportError:          # tracker degrades gracefully without OpenCV
    cv2 = None


def cluster_features(pts, idxs, ranges, range_min, range_max):
    """17-feature vector for one cluster.

    pts: (n,2) cluster points in the LASER frame (order = scan order)
    idxs: original beam indices of those points
    ranges: full scan ranges array
    """
    pts = np.asarray(pts, dtype=np.float64)
    n = len(pts)
    mean = pts.mean(axis=0)
    median = np.median(pts, axis=0)
    distance = float(np.hypot(*median))

    d2_mean = ((pts - mean) ** 2).sum(axis=1)
    std = float(np.sqrt(d2_mean.sum() / (n - 1.0)))
    avg_median_dev = float(np.linalg.norm(pts - median, axis=1).mean())

    # occlusion flags: is the first/last point's neighbor beam a valid
    # return that is NEARER than the cluster edge?
    def occluded(edge_i, edge_pt_range, neighbor_i):
        if not (0 <= neighbor_i < len(ranges)):
            return 1.0
        nr = ranges[neighbor_i]
        if not np.isfinite(nr) or nr < range_min or nr > range_max:
            return 1.0
        return 0.0 if (edge_pt_range < nr or nr < 0.01) else 1.0

    r_first = float(ranges[idxs[0]])
    r_last = float(ranges[idxs[-1]])
    occluded_left = occluded(idxs[0], r_first, idxs[0] - 1)
    occluded_right = occluded(idxs[-1], r_last, idxs[-1] + 1)

    width = float(np.linalg.norm(pts[0] - pts[-1]))

    # linearity: residual energy off the first principal axis
    s = np.linalg.svd(pts - mean, compute_uv=False)
    linearity = float(s[1] ** 2) if len(s) > 1 else 0.0

    # circularity: residuals of a least-squares circle fit
    A = np.column_stack([-2.0 * pts[:, 0], -2.0 * pts[:, 1], np.ones(n)])
    B = -(pts[:, 0] ** 2 + pts[:, 1] ** 2)
    sol, *_ = np.linalg.lstsq(A, B, rcond=None)
    xc, yc = sol[0], sol[1]
    rc = float(np.sqrt(max(0.0, xc * xc + yc * yc - sol[2])))
    circularity = float(((rc - np.hypot(xc - pts[:, 0], yc - pts[:, 1])) ** 2).sum())

    # consecutive-triplet pass: boundary length/regularity, curvature, angle
    with np.errstate(divide="ignore", invalid="ignore"):
        ml = pts[2:] - pts[1:-1]          # mid -> left
        mr = pts[:-2] - pts[1:-1]         # mid -> right
        lr = pts[2:] - pts[:-2]
        L_ml = np.linalg.norm(ml, axis=1)
        L_mr = np.linalg.norm(mr, axis=1)
        L_lr = np.linalg.norm(lr, axis=1)

        boundary_length = float(L_mr.sum() + L_ml[-1])
        sum_b_sq = float((L_mr ** 2).sum() + L_ml[-1] ** 2)
        boundary_regularity = float(np.sqrt(max(
            0.0, (sum_b_sq - boundary_length ** 2 / n) / (n - 1.0))))

        a = (ml * mr).sum(axis=1) / L_mr ** 2
        b = (ml[:, 0] * mr[:, 1] - ml[:, 1] * mr[:, 0]) / L_mr ** 2
        th = np.arctan2(b, a)
        th = np.where(th < 0, th + 2 * np.pi, th)
        ang_diff = float(np.nansum(th) / n)

        sh = 0.5 * (L_ml + L_mr + L_lr)
        area = np.sqrt(np.maximum(0.0, sh * (sh - L_ml) * (sh - L_mr) * (sh - L_lr)))
        curv = 4.0 * area / (L_ml * L_mr * L_lr * n)
        mean_curvature = float(np.nansum(np.where(th > 0, curv, -curv)))

        # inscribed angle variance: angle subtended at each interior point
        # by the cluster endpoints
        ml2 = pts[0] - pts[1:-1]
        mr2 = pts[-1] - pts[1:-1]
        L2 = (mr2 ** 2).sum(axis=1)
        a2 = (ml2 * mr2).sum(axis=1) / L2
        b2 = (ml2[:, 0] * mr2[:, 1] - ml2[:, 1] * mr2[:, 0]) / L2
        th2 = np.arctan2(b2, a2)
        th2 = np.where(th2 < 0, th2 + 2 * np.pi, th2)
        sum_iav = float(np.nansum(th2))
        sum_iav_sq = float(np.nansum(th2 ** 2))
        iav = sum_iav / n
        std_iav = float(np.sqrt(max(0.0, (sum_iav_sq - sum_iav ** 2 / n) / (n - 1.0))))

    feats = np.array([
        n, std, avg_median_dev, width, linearity, circularity,
        0.0,                       # radius: hardcoded 0 in training code
        boundary_length, boundary_regularity, mean_curvature, ang_diff,
        iav, std_iav, distance, distance / n, occluded_right, occluded_left,
    ], dtype=np.float32)
    return np.nan_to_num(feats, nan=0.0, posinf=0.0, neginf=0.0)


class LegClassifier:
    """Loads the pretrained forest; score() -> P(leg) in [0, 1]."""

    def __init__(self, forest_file):
        self.forest = None
        self.pos_col = None
        if cv2 is None:
            return
        try:
            self.forest = cv2.ml.RTrees_load(forest_file)
            probe = np.zeros((1, 17), dtype=np.float32)
            out = self._votes(probe)
            self.pos_col = int(np.where(out[0] == 1)[0][0])
        except Exception:
            self.forest = None

    def _votes(self, samples):
        # cv2 binding returns just the matrix (row 0 = class labels,
        # row 1+ = votes per sample); some builds return (retval, matrix)
        out = self.forest.getVotes(samples, 0)
        if isinstance(out, tuple):
            out = out[-1]
        return out

    @property
    def ok(self):
        return self.forest is not None

    def score(self, pts, idxs, ranges, range_min, range_max):
        if self.forest is None or len(pts) < 3:
            return 0.5
        feats = cluster_features(pts, idxs, ranges, range_min, range_max)
        out = self._votes(feats.reshape(1, -1))
        votes = out[1].astype(np.float64)
        return float(votes[self.pos_col] / votes.sum())
