"""Pure geometry for interactive point-cloud frame alignment."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np


EPS = 1e-9


@dataclass
class Line2D:
    normal: np.ndarray
    c: float
    p0: np.ndarray
    p1: np.ndarray
    theta: float
    length: float
    score: float


@dataclass
class CornerCandidate:
    xy: np.ndarray
    score: float
    line_i: int
    line_j: int


@dataclass
class RankedOrigin:
    xy: np.ndarray
    score: float
    line_score: float
    positive_ratio: float
    x_axis_angle_rad: float
    x_axis_2d: np.ndarray
    y_axis_2d: np.ndarray
    source_candidate: CornerCandidate


def require_finite(name: str, values: np.ndarray) -> None:
    if not np.isfinite(values).all():
        raise ValueError(f"{name} must be finite.")


def normalize(v: np.ndarray, name: str = "vector") -> np.ndarray:
    norm = np.linalg.norm(v)
    if norm < EPS:
        raise ValueError(f"Cannot normalize near-zero {name}.")
    return v / norm


def angle_mod_pi(theta: float) -> float:
    theta = theta % math.pi
    return theta + math.pi if theta < 0 else theta


def angle_diff_mod_pi(a: float, b: float) -> float:
    """Return the smallest angle difference for unoriented 2D lines."""
    return abs((a - b + math.pi / 2.0) % math.pi - math.pi / 2.0)


def line_from_two_points(
    first: np.ndarray,
    second: np.ndarray,
) -> tuple[np.ndarray, float] | None:
    direction = second - first
    norm = np.linalg.norm(direction)
    if norm < EPS:
        return None
    direction = direction / norm
    normal = np.array([-direction[1], direction[0]], dtype=float)
    return normal, -float(np.dot(normal, first))


def distances_to_line(
    xy: np.ndarray,
    normal: np.ndarray,
    c: float,
) -> np.ndarray:
    return np.abs(xy @ normal + c)


def fit_line_pca(xy: np.ndarray) -> Line2D:
    """Refit a line to inlier points using PCA/SVD."""
    if len(xy) < 2:
        raise ValueError("Need at least two points to fit a line.")

    centroid = np.mean(xy, axis=0)
    centered = xy - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    direction = normalize(vh[0], "line direction")
    theta = angle_mod_pi(math.atan2(direction[1], direction[0]))
    direction = np.array([math.cos(theta), math.sin(theta)], dtype=float)
    normal = np.array([-direction[1], direction[0]], dtype=float)
    c = -float(np.dot(normal, centroid))
    projections = centered @ direction
    p0 = centroid + np.min(projections) * direction
    p1 = centroid + np.max(projections) * direction
    length = float(np.linalg.norm(p1 - p0))

    return Line2D(
        normal=normal,
        c=c,
        p0=p0,
        p1=p1,
        theta=theta,
        length=length,
        score=float(len(xy) * max(length, EPS)),
    )


def ransac_detect_lines(
    xy: np.ndarray,
    threshold: float,
    min_inliers: int,
    max_lines: int,
    iterations: int,
    seed: int,
) -> list[Line2D]:
    """Iteratively detect 2D lines with RANSAC."""
    if len(xy) < min_inliers:
        return []

    rng = np.random.default_rng(seed)
    remaining = xy
    lines: list[Line2D] = []

    for line_idx in range(max_lines):
        if len(remaining) < min_inliers:
            break

        best_inliers_mask = None
        best_count = 0
        for _ in range(iterations):
            ids = rng.choice(len(remaining), size=2, replace=False)
            model = line_from_two_points(remaining[ids[0]], remaining[ids[1]])
            if model is None:
                continue
            normal, c = model
            mask = distances_to_line(remaining, normal, c) < threshold
            count = int(np.count_nonzero(mask))
            if count > best_count:
                best_count = count
                best_inliers_mask = mask

        if best_inliers_mask is None or best_count < min_inliers:
            break

        refined = fit_line_pca(remaining[best_inliers_mask])
        if refined.length > threshold * 8.0:
            lines.append(refined)
        remaining = remaining[~best_inliers_mask]
        print(
            f"Detected line {line_idx + 1}: "
            f"inliers={best_count}, length={refined.length:.3f}, "
            f"theta={math.degrees(refined.theta):.2f} deg"
        )

    return lines


def estimate_manhattan_base_angle(lines: list[Line2D]) -> float:
    """Estimate dominant orientation with orthogonal lines considered equal."""
    if not lines:
        raise ValueError("No lines available for Manhattan angle estimation.")
    weights = np.array([max(line.score, EPS) for line in lines], dtype=float)
    thetas = np.array([line.theta for line in lines], dtype=float)
    sine = np.sum(weights * np.sin(4.0 * thetas))
    cosine = np.sum(weights * np.cos(4.0 * thetas))
    return (0.25 * math.atan2(sine, cosine)) % (math.pi / 2.0)


def line_group(line: Line2D, base_angle: float) -> str:
    base_difference = angle_diff_mod_pi(line.theta, base_angle)
    perpendicular_difference = angle_diff_mod_pi(
        line.theta,
        base_angle + math.pi / 2.0,
    )
    return (
        "x_parallel"
        if base_difference <= perpendicular_difference
        else "y_parallel"
    )


def intersect_lines(first: Line2D, second: Line2D) -> np.ndarray | None:
    matrix = np.vstack([first.normal, second.normal])
    if abs(np.linalg.det(matrix)) < 1e-6:
        return None
    return np.linalg.solve(matrix, -np.array([first.c, second.c], dtype=float))


def generate_corner_candidates(
    lines: list[Line2D],
    base_angle: float,
    angle_tolerance_deg: float,
) -> list[CornerCandidate]:
    tolerance = math.radians(angle_tolerance_deg)
    x_lines: list[tuple[int, Line2D]] = []
    y_lines: list[tuple[int, Line2D]] = []

    for index, line in enumerate(lines):
        group = line_group(line, base_angle)
        expected = (
            base_angle
            if group == "x_parallel"
            else base_angle + math.pi / 2.0
        )
        if angle_diff_mod_pi(line.theta, expected) <= tolerance:
            lines_for_group = x_lines if group == "x_parallel" else y_lines
            lines_for_group.append((index, line))

    candidates: list[CornerCandidate] = []
    for first_index, x_line in x_lines:
        for second_index, y_line in y_lines:
            xy = intersect_lines(x_line, y_line)
            if xy is None or not np.all(np.isfinite(xy)):
                continue
            angle_quality = 1.0 - min(
                angle_diff_mod_pi(
                    x_line.theta,
                    y_line.theta + math.pi / 2.0,
                )
                / max(tolerance, EPS),
                1.0,
            )
            candidates.append(
                CornerCandidate(
                    xy=xy,
                    score=float(
                        (x_line.score + y_line.score)
                        * max(angle_quality, 0.1)
                    ),
                    line_i=first_index,
                    line_j=second_index,
                )
            )
    return candidates


def merge_close_candidates(
    candidates: list[CornerCandidate],
    radius: float,
) -> list[CornerCandidate]:
    merged: list[CornerCandidate] = []
    for candidate in sorted(
        candidates,
        key=lambda item: item.score,
        reverse=True,
    ):
        for existing in merged:
            if np.linalg.norm(candidate.xy - existing.xy) < radius:
                existing.score += candidate.score
                break
        else:
            merged.append(candidate)
    return sorted(merged, key=lambda item: item.score, reverse=True)


def best_axes_for_origin(
    xy_points: np.ndarray,
    origin_xy: np.ndarray,
    base_angle: float,
    positive_margin: float,
) -> tuple[float, np.ndarray, np.ndarray, float]:
    centered = xy_points - origin_xy
    best: tuple[float, np.ndarray, np.ndarray, float, float] | None = None
    for angle in (
        base_angle,
        base_angle + math.pi,
        base_angle + math.pi / 2.0,
        base_angle + 3.0 * math.pi / 2.0,
    ):
        x_axis = np.array([math.cos(angle), math.sin(angle)], dtype=float)
        y_axis = np.array([-x_axis[1], x_axis[0]], dtype=float)
        local_x = centered @ x_axis
        local_y = centered @ y_axis
        positive_ratio = float(
            np.mean(
                (local_x >= -positive_margin)
                & (local_y >= -positive_margin)
            )
        )
        negative_extent = max(
            0.0,
            -float(np.percentile(local_x, 1.0)),
            -float(np.percentile(local_y, 1.0)),
        )
        score = positive_ratio * 1000.0 - negative_extent
        if best is None or score > best[0]:
            best = (score, x_axis, y_axis, positive_ratio, angle)

    assert best is not None
    _, x_axis, y_axis, positive_ratio, angle = best
    return angle, x_axis, y_axis, positive_ratio


def rank_origin_candidates(
    xy_points: np.ndarray,
    candidates: list[CornerCandidate],
    base_angle: float,
    positive_margin: float,
) -> list[RankedOrigin]:
    if not candidates:
        return []

    max_line_score = max(candidate.score for candidate in candidates)
    ranked = []
    for candidate in candidates:
        angle, x_axis, y_axis, positive_ratio = best_axes_for_origin(
            xy_points,
            candidate.xy,
            base_angle,
            positive_margin,
        )
        ranked.append(
            RankedOrigin(
                xy=candidate.xy,
                score=float(
                    positive_ratio * 10.0
                    + candidate.score / max(max_line_score, EPS)
                ),
                line_score=float(candidate.score),
                positive_ratio=float(positive_ratio),
                x_axis_angle_rad=float(angle),
                x_axis_2d=x_axis,
                y_axis_2d=y_axis,
                source_candidate=candidate,
            )
        )
    return sorted(ranked, key=lambda item: item.score, reverse=True)


def make_ranked_origin_from_manual_click(
    xy_points: np.ndarray,
    origin_xy: np.ndarray,
    base_angle: float,
    positive_margin: float,
    x_axis_hint_xy: np.ndarray | None = None,
) -> RankedOrigin:
    if x_axis_hint_xy is not None:
        x_axis = normalize(x_axis_hint_xy - origin_xy, "manual x-axis hint")
        angle = math.atan2(x_axis[1], x_axis[0])
        y_axis = np.array([-x_axis[1], x_axis[0]], dtype=float)
        centered = xy_points - origin_xy
        local_x = centered @ x_axis
        local_y = centered @ y_axis
        positive_ratio = float(
            np.mean(
                (local_x >= -positive_margin)
                & (local_y >= -positive_margin)
            )
        )
    else:
        angle, x_axis, y_axis, positive_ratio = best_axes_for_origin(
            xy_points,
            origin_xy,
            base_angle,
            positive_margin,
        )

    candidate = CornerCandidate(origin_xy, 0.0, -1, -1)
    return RankedOrigin(
        xy=origin_xy,
        score=9999.0,
        line_score=0.0,
        positive_ratio=positive_ratio,
        x_axis_angle_rad=float(angle),
        x_axis_2d=x_axis,
        y_axis_2d=y_axis,
        source_candidate=candidate,
    )


def build_global_to_local_transform(
    origin: np.ndarray,
    x_axis: np.ndarray,
    z_axis: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Build a transform mapping homogeneous global points into local frame."""
    z_axis = normalize(z_axis.astype(float), "z_axis")
    x_axis = normalize(x_axis.astype(float), "x_axis")
    x_axis = x_axis - np.dot(x_axis, z_axis) * z_axis
    x_axis = normalize(x_axis, "x_axis projected perpendicular to z_axis")
    y_axis = normalize(np.cross(z_axis, x_axis), "y_axis")
    rotation = np.vstack([x_axis, y_axis, z_axis])
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = -rotation @ origin
    return transform, x_axis, y_axis, z_axis


def compute_diagnostics(
    transform: np.ndarray,
    x_axis: np.ndarray,
    y_axis: np.ndarray,
    z_axis: np.ndarray,
) -> dict[str, Any]:
    rotation = transform[:3, :3]
    return {
        "det_R": float(np.linalg.det(rotation)),
        "dot_x_y": float(np.dot(x_axis, y_axis)),
        "dot_x_z": float(np.dot(x_axis, z_axis)),
        "dot_y_z": float(np.dot(y_axis, z_axis)),
    }
