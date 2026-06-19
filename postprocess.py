from __future__ import annotations
import argparse
import csv
import math
import sys
from pathlib import Path
import numpy as np

try:
    import open3d as o3d

    HAS_O3D = True
except ImportError:
    HAS_O3D = False
    print(
        "WARNING: Open3D is not installed. High-performance filtering (Voxel, SOR) will be limited."
    )
import os

os.environ.setdefault("MPLCONFIGDIR", str((Path.cwd() / ".matplotlib-cache").resolve()))
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.colors as mcolors
from mpl_toolkits.mplot3d import Axes3D


def load_csv(path: str | Path) -> dict:
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"CSV 파일 없음: {path}")
    raw = np.genfromtxt(
        path,
        delimiter=",",
        names=True,
        dtype=None,
        encoding="utf-8",
        autostrip=True,
    )
    headers = list(raw.dtype.names)
    norm = {h.strip().lower(): h for h in headers}

    def find(aliases):
        for a in aliases:
            if a.lower() in norm:
                return raw[norm[a.lower()]].astype(np.float64)
        return None

    data = {}
    data["x"] = find(["x_cm", "X_cm"])
    data["y"] = find(["y_cm", "Y_cm"])
    data["z"] = find(["z_cm", "Z_cm"])
    data["yaw"] = find(["yaw_deg", "angle_deg"])
    data["pitch"] = find(["pitch_deg"])
    data["dist"] = find(["distance_cm", "dist_cm"])
    data["strength"] = find(["strength", "signal"])
    if data["x"] is None or data["y"] is None:
        raise ValueError(f"CSV에 x_cm, y_cm 컬럼이 필요합니다. 발견된 헤더: {headers}")
    if data["z"] is None:
        data["z"] = np.zeros_like(data["x"])
    if data["dist"] is None:
        data["dist"] = np.sqrt(data["x"] ** 2 + data["y"] ** 2 + data["z"] ** 2)
    if data["strength"] is None:
        data["strength"] = np.ones_like(data["x"])
    if data["yaw"] is None:
        data["yaw"] = np.zeros_like(data["x"])
    if data["pitch"] is None:
        data["pitch"] = np.zeros_like(data["x"])
    print("[main] Recalculating correct XYZ coordinates from raw yaw/pitch/dist...")
    yaw_rad = np.radians(data["yaw"]) * -1.0
    pitch_rad = np.radians(data["pitch"])
    r_horizontal = data["dist"] * np.cos(pitch_rad)
    actual_radius = 48.0 - r_horizontal
    data["x"] = actual_radius * np.sin(yaw_rad)
    data["y"] = actual_radius * np.cos(yaw_rad)
    data["z"] = data["dist"] * np.sin(pitch_rad)
    data["n"] = len(data["x"])
    return data


def zbuffer_filter(
    data: dict,
    yaw_bin_deg: float = 1.0,
    pitch_bin_deg: float = 0.5,
) -> np.ndarray:
    yaw = data["yaw"]
    pitch = data["pitch"]
    dist = data["dist"]
    N = data["n"]
    yaw_idx = np.round(yaw / yaw_bin_deg).astype(np.int32)
    pitch_idx = np.round(pitch / pitch_bin_deg).astype(np.int32)
    cell_key = yaw_idx.astype(np.int64) * 1_000_000 + pitch_idx.astype(np.int64)
    inlier_mask = np.zeros(N, dtype=bool)
    order = np.argsort(cell_key, kind="stable")
    sorted_keys = cell_key[order]
    sorted_dist = dist[order]
    boundaries = np.concatenate(([0], np.where(np.diff(sorted_keys) != 0)[0] + 1, [N]))
    for start, end in zip(boundaries[:-1], boundaries[1:]):
        local_min = np.argmin(sorted_dist[start:end])
        inlier_mask[order[start + local_min]] = True
    n_removed = N - inlier_mask.sum()
    print(
        f"[zBuffer] {inlier_mask.sum():,} points kept, {n_removed:,} ghost points removed "
        f"(bin: yaw={yaw_bin_deg}°, pitch={pitch_bin_deg}°)"
    )
    return inlier_mask


from scipy.interpolate import griddata


def spherical_hole_fill(
    data: dict,
    yaw_bin_deg: float = 2.0,
    pitch_bin_deg: float = 0.6,
) -> tuple[dict, int]:
    yaw = data["yaw"]
    pitch = data["pitch"]
    dist = data["dist"]
    strength = data["strength"]
    yaw_idx = np.round(yaw / yaw_bin_deg).astype(np.int32)
    pitch_idx = np.round(pitch / pitch_bin_deg).astype(np.int32)
    yaw_min, yaw_max = yaw_idx.min(), yaw_idx.max()
    pitch_min, pitch_max = pitch_idx.min(), pitch_idx.max()
    grid = {}
    for i in range(len(yaw)):
        key = (int(yaw_idx[i]), int(pitch_idx[i]))
        if key not in grid:
            grid[key] = [0.0, 0.0, 0]
        grid[key][0] += dist[i]
        grid[key][1] += strength[i]
        grid[key][2] += 1
    grid_avg = {}
    for key, (sd, ss, cnt) in grid.items():
        grid_avg[key] = (sd / cnt, ss / cnt)
    pts = np.array(list(grid_avg.keys()))
    vals_d = np.array([v[0] for v in grid_avg.values()])
    vals_s = np.array([v[1] for v in grid_avg.values()])
    yaw_span = int(np.round(360.0 / yaw_bin_deg))
    pts_left = pts.copy()
    pts_left[:, 0] -= yaw_span
    pts_right = pts.copy()
    pts_right[:, 0] += yaw_span
    all_pts = np.vstack([pts, pts_left, pts_right])
    all_vals_d = np.concatenate([vals_d, vals_d, vals_d])
    all_vals_s = np.concatenate([vals_s, vals_s, vals_s])
    target_yi = []
    target_pi = []
    for yi in range(yaw_min, yaw_max + 1):
        for pi in range(pitch_min, pitch_max + 1):
            if (yi, pi) not in grid_avg:
                target_yi.append(yi)
                target_pi.append(pi)
    if not target_yi:
        print(
            f"[HoleFill] 채울 빈 셀 없음 (격자: yaw={yaw_bin_deg}°, pitch={pitch_bin_deg}°)"
        )
        return data, 0
    target_pts = np.column_stack([target_yi, target_pi])
    interp_d = griddata(all_pts, all_vals_d, target_pts, method="linear")
    interp_s = griddata(all_pts, all_vals_s, target_pts, method="linear")
    valid_mask = ~np.isnan(interp_d)
    new_yaw = np.array(target_yi)[valid_mask] * yaw_bin_deg
    new_pitch = np.array(target_pi)[valid_mask] * pitch_bin_deg
    new_dist = interp_d[valid_mask]
    new_strength = interp_s[valid_mask]
    n_filled = len(new_yaw)
    if n_filled == 0:
        print(
            f"[HoleFill] 채울 빈 셀 없음 (격자: yaw={yaw_bin_deg}°, pitch={pitch_bin_deg}°)"
        )
        return data, 0
    new_yaw = np.array(new_yaw)
    new_pitch = np.array(new_pitch)
    new_dist = np.array(new_dist)
    new_strength = np.array(new_strength)
    yaw_rad = np.radians(new_yaw) * -1.0
    pitch_rad = np.radians(new_pitch)
    r_horizontal = new_dist * np.cos(pitch_rad)
    actual_radius = 48.0 - r_horizontal
    new_x = actual_radius * np.sin(yaw_rad)
    new_y = actual_radius * np.cos(yaw_rad)
    new_z = new_dist * np.sin(pitch_rad)
    for key in ["yaw", "pitch", "dist", "strength", "x", "y", "z"]:
        old = data[key]
        if key == "yaw":
            new_arr = new_yaw
        elif key == "pitch":
            new_arr = new_pitch
        elif key == "dist":
            new_arr = new_dist
        elif key == "strength":
            new_arr = new_strength
        elif key == "x":
            new_arr = new_x
        elif key == "y":
            new_arr = new_y
        elif key == "z":
            new_arr = new_z
        else:
            continue
        data[key] = np.concatenate([old, new_arr])
    data["n"] = len(data["x"])
    print(
        f"[HoleFill] {n_filled:,} 보간 포인트 추가 "
        f"(격자: yaw={yaw_bin_deg}°, pitch={pitch_bin_deg}°, "
        f"Convex Hull 무조건 채움)"
    )
    return data, n_filled


def compute_adaptive_sor_params(
    data: dict,
    base_k: int = 8,
    base_voxel: float = 0.5,
    reference_pitch_res_cm: float = 0.5,
) -> tuple[int, float]:
    pitch = data["pitch"]
    unique_pitch = np.unique(np.round(pitch, 2))
    if len(unique_pitch) < 2:
        return base_k, base_voxel
    delta_pitch_deg = np.median(np.diff(np.sort(unique_pitch)))
    pitch_mean_rad = np.radians(np.median(np.abs(unique_pitch)))
    TARGET_DISTANCE_CM = 48.0
    estimated_res_cm = abs(delta_pitch_deg * np.pi / 180.0 * TARGET_DISTANCE_CM)
    estimated_res_cm = max(estimated_res_cm, 0.1)
    ratio = estimated_res_cm / reference_pitch_res_cm
    adaptive_k = max(4, int(round(base_k / ratio)))
    adaptive_voxel = float(np.clip(base_voxel * ratio, 0.2, 5.0))
    print(
        f"[AdaptiveSOR] 추정 pitch 해상도: {estimated_res_cm:.2f} cm "
        f"(기준: {reference_pitch_res_cm} cm, 비율: {ratio:.2f})"
    )
    print(
        f"[AdaptiveSOR] 자동 파라미터: k={adaptive_k}, voxel_size={adaptive_voxel:.2f} cm"
    )
    return adaptive_k, adaptive_voxel


def statistical_outlier_removal(
    points: np.ndarray,
    k: int = 8,
    std_ratio: float = 1.5,
    voxel_size: float = 0.5,
) -> tuple[np.ndarray, np.ndarray]:
    if not HAS_O3D:
        print("Open3D가 없어 원본 데이터를 그대로 반환합니다. 설치: pip install open3d")
        N = points.shape[0]
        return np.ones(N, dtype=bool), np.zeros(N)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    print(f"  [Open3D] Voxel Downsampling (size={voxel_size}cm)...")
    pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
    print(f"  [Open3D] SOR Filter (k={k}, std_ratio={std_ratio})...")
    pcd_sor, ind_sor = pcd_down.remove_statistical_outlier(
        nb_neighbors=k, std_ratio=std_ratio
    )
    print("  [Open3D] Radius Outlier Removal...")
    pcd_final, ind_rad = pcd_sor.remove_radius_outlier(nb_points=4, radius=3.0)
    filtered_points = np.asarray(pcd_final.points)
    print("  [Open3D] Masking original points...")
    if len(filtered_points) == 0:
        return np.zeros(points.shape[0], dtype=bool), np.zeros(points.shape[0])
    from scipy.spatial import cKDTree

    tree = cKDTree(filtered_points)
    distances, _ = tree.query(points, k=1, workers=-1)
    inlier_mask = distances < voxel_size
    return inlier_mask, np.zeros(points.shape[0])


def compute_bounding_box(points: np.ndarray) -> dict:
    pmin = points.min(axis=0)
    pmax = points.max(axis=0)
    size = pmax - pmin
    center = (pmin + pmax) / 2.0
    volume = np.prod(size) if np.all(size > 0) else 0
    return {
        "min": pmin,
        "max": pmax,
        "size": size,
        "center": center,
        "volume": volume,
    }


def draw_bounding_box(ax, bbox: dict, color="#00cec9", linewidth=1.5):
    mn = bbox["min"]
    mx = bbox["max"]
    corners = np.array(
        [
            [mn[0], mn[1], mn[2]],
            [mx[0], mn[1], mn[2]],
            [mx[0], mx[1], mn[2]],
            [mn[0], mx[1], mn[2]],
            [mn[0], mn[1], mx[2]],
            [mx[0], mn[1], mx[2]],
            [mx[0], mx[1], mx[2]],
            [mn[0], mx[1], mx[2]],
        ]
    )
    edges = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ]
    for i, j in edges:
        ax.plot3D(
            [corners[i, 0], corners[j, 0]],
            [corners[i, 1], corners[j, 1]],
            [corners[i, 2], corners[j, 2]],
            color=color,
            linewidth=linewidth,
            alpha=0.8,
        )


def cross_section(
    points: np.ndarray,
    z_value: float,
    thickness: float = 1.0,
) -> np.ndarray:
    half = thickness / 2.0
    mask = (points[:, 2] >= z_value - half) & (points[:, 2] <= z_value + half)
    return points[mask]


def compute_material_segmentation(
    reflectivity: np.ndarray, min_bins: int = 2, max_bins: int = 5
) -> np.ndarray:
    if len(reflectivity) < 10:
        return np.zeros(len(reflectivity), dtype=int)
    try:
        from scipy.signal import find_peaks
    except ImportError:
        print("WARNING: scipy is not installed. Using median split.")
        return np.digitize(reflectivity, [np.median(reflectivity)])
    hist, bin_edges = np.histogram(reflectivity, bins=50)
    bin_centers = (bin_edges[:-1] + bin_edges[1:]) / 2.0
    window = np.ones(5) / 5.0
    smooth_hist = np.convolve(hist, window, mode="same")
    min_prominence = len(reflectivity) * 0.01
    peaks, _ = find_peaks(smooth_hist, distance=4, prominence=min_prominence)
    n_clusters = len(peaks)
    if n_clusters < min_bins:
        thresholds = np.percentile(
            reflectivity, np.linspace(0, 100, min_bins + 1)[1:-1]
        )
    else:
        if n_clusters > max_bins:
            peak_heights = smooth_hist[peaks]
            largest_peaks_idx = np.argsort(peak_heights)[-max_bins:]
            peaks = np.sort(peaks[largest_peaks_idx])
        thresholds = []
        for i in range(len(peaks) - 1):
            p1 = peaks[i]
            p2 = peaks[i + 1]
            mid_idx = (p1 + p2) // 2
            thresholds.append(bin_centers[mid_idx])
    labels = np.digitize(reflectivity, thresholds)
    print(
        f"[Segmentation] 동적 히스토그램 분석: {len(thresholds)+1}개의 재질 레이어 식별"
    )
    return labels


def reconstruct_surface_mesh(points: np.ndarray):
    if not HAS_O3D or len(points) < 10:
        return None
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=3.0, max_nn=30)
    )
    pcd.orient_normals_consistent_tangent_plane(100)
    try:
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd, depth=8
        )
        densities = np.asarray(densities)
        density_threshold = np.percentile(densities, 5)
        vertices_to_remove = densities < density_threshold
        mesh.remove_vertices_by_mask(vertices_to_remove)
        return mesh
    except Exception as e:
        print(f"Mesh reconstruction failed: {e}")
        return None


def compute_occupancy_grid(points: np.ndarray, voxel_size: float = 1.0):
    if not HAS_O3D or len(points) < 1:
        return None
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=voxel_size)


def render_analysis(
    data: dict,
    filtered_points: np.ndarray,
    original_points: np.ndarray,
    inlier_mask: np.ndarray,
    bbox: dict,
    slice_z: float,
    slice_pts: np.ndarray,
    out_png: str = "postprocess_result.png",
):
    fig = plt.figure(figsize=(24, 14))
    fig.patch.set_facecolor("#0d0d10")
    fig.suptitle(
        "Advanced LiDAR Point Cloud Analysis",
        color="#ffffff",
        fontsize=18,
        fontweight="bold",
        y=0.96,
    )
    cmap = "plasma"
    point_size = 6
    alpha = 0.7
    ax1 = fig.add_subplot(231, projection="3d")
    ax1.set_facecolor("#0d0d10")
    ax1.set_title("1. Original + Outliers", color="#ffffff", pad=10, fontsize=12)
    outlier_mask = ~inlier_mask
    inlier_pts = original_points[inlier_mask]
    outlier_pts = original_points[outlier_mask]
    if len(inlier_pts) > 0:
        norm_z = inlier_pts[:, 2] - inlier_pts[:, 2].min()
        span = inlier_pts[:, 2].max() - inlier_pts[:, 2].min()
        if span < 1e-9:
            span = 1.0
        norm_z = norm_z / span
        colors_in = plt.get_cmap(cmap)(norm_z)
        ax1.scatter(
            inlier_pts[:, 0],
            inlier_pts[:, 1],
            inlier_pts[:, 2],
            c=colors_in,
            s=point_size,
            alpha=alpha,
            linewidths=0,
        )
    if len(outlier_pts) > 0:
        ax1.scatter(
            outlier_pts[:, 0],
            outlier_pts[:, 1],
            outlier_pts[:, 2],
            c="#e17055",
            s=point_size * 2,
            alpha=0.9,
            marker="x",
            linewidths=1.0,
            label=f"Outliers ({len(outlier_pts)})",
        )
        ax1.legend(
            loc="upper right",
            fontsize=8,
            facecolor="#1a1a24",
            edgecolor="#333",
            labelcolor="#ccc",
        )
    _style_3d_ax(ax1)
    if len(filtered_points) > 0:
        raw_strength = data["strength"][inlier_mask]
        dist_cm = data["dist"][inlier_mask]
        valid_mask = (raw_strength >= 100) & (raw_strength < 65530)
        reflectivity = np.zeros_like(raw_strength, dtype=float)
        reflectivity[valid_mask] = raw_strength[valid_mask] * (dist_cm[valid_mask] ** 2)
        if np.any(valid_mask):
            reflectivity[~valid_mask] = np.median(reflectivity[valid_mask])
        s_min, s_max = np.percentile(reflectivity, 5), np.percentile(reflectivity, 95)
        if s_min >= s_max:
            s_min, s_max = 0, 1
        norm_s = np.clip((reflectivity - s_min) / (s_max - s_min), 0, 1)
    ax2 = fig.add_subplot(232, projection="3d")
    ax2.set_facecolor("#0d0d10")
    ax2.set_title(
        f"2. Filtered ({len(filtered_points):,} pts) & Box",
        color="#ffffff",
        pad=10,
        fontsize=12,
    )
    if len(filtered_points) > 0:
        intensity_cmap = "magma"
        colors_f = plt.get_cmap(intensity_cmap)(norm_s)
        scalar_map = cm.ScalarMappable(
            cmap=plt.get_cmap(intensity_cmap),
            norm=mcolors.Normalize(vmin=s_min, vmax=s_max),
        )
        scalar_map.set_array([])
        ax2.scatter(
            filtered_points[:, 0],
            filtered_points[:, 1],
            filtered_points[:, 2],
            c=colors_f,
            s=point_size,
            alpha=alpha,
            linewidths=0,
        )
        draw_bounding_box(ax2, bbox)
        s = bbox["size"]
        dim_text = f"{s[0]:.1f} × {s[1]:.1f} × {s[2]:.1f} cm"
        ax2.text2D(
            0.02,
            0.02,
            dim_text,
            transform=ax2.transAxes,
            color="#00cec9",
            fontsize=10,
            fontweight="bold",
            bbox=dict(
                boxstyle="round,pad=0.3",
                facecolor="#1a1a24",
                edgecolor="#00cec9",
                alpha=0.8,
            ),
        )
    _style_3d_ax(ax2)
    ax3 = fig.add_subplot(233, projection="3d")
    ax3.set_facecolor("#0d0d10")
    ax3.set_title(
        "3. Dynamic Peak Material Segmentation", color="#ffffff", pad=10, fontsize=12
    )
    if len(filtered_points) > 0:
        labels = compute_material_segmentation(norm_s)
        unique_labels = np.unique(labels)
        cmap_seg = plt.get_cmap("tab10")
        seg_colors = [cmap_seg(i % 10) for i in range(max(unique_labels) + 1)]
        colors_seg = [seg_colors[l] for l in labels]
        ax3.scatter(
            filtered_points[:, 0],
            filtered_points[:, 1],
            filtered_points[:, 2],
            c=colors_seg,
            s=point_size * 2,
            alpha=0.9,
            linewidths=0,
        )
        from matplotlib.lines import Line2D

        legend_elements = []
        for i in sorted(list(unique_labels)):
            label_name = f"Material {i+1}"
            if i == min(unique_labels):
                label_name += " (Dark)"
            if i == max(unique_labels) and len(unique_labels) > 1:
                label_name += " (Bright)"
            legend_elements.append(
                Line2D(
                    [0],
                    [0],
                    marker="o",
                    color="w",
                    markerfacecolor=seg_colors[i],
                    markersize=8,
                    label=label_name,
                )
            )
        ax3.legend(
            handles=legend_elements,
            loc="upper right",
            fontsize=8,
            facecolor="#1a1a24",
            edgecolor="#333",
            labelcolor="#ccc",
        )
    _style_3d_ax(ax3)
    ax4 = fig.add_subplot(234, projection="3d")
    ax4.set_facecolor("#0d0d10")
    ax4.set_title("4. 3D Voxel Occupancy Grid", color="#ffffff", pad=10, fontsize=12)
    if len(filtered_points) > 0:
        voxel_grid = compute_occupancy_grid(filtered_points, voxel_size=1.5)
        if voxel_grid is not None:
            voxels = voxel_grid.get_voxels()
            voxel_indices = np.array([v.grid_index for v in voxels])
            if len(voxel_indices) > 0:
                origin = voxel_grid.origin
                voxel_size = voxel_grid.voxel_size
                voxel_centers = origin + voxel_indices * voxel_size + (voxel_size / 2)
                norm_vz = (voxel_centers[:, 2] - voxel_centers[:, 2].min()) / (
                    voxel_centers[:, 2].max() - voxel_centers[:, 2].min() + 1e-9
                )
                ax4.scatter(
                    voxel_centers[:, 0],
                    voxel_centers[:, 1],
                    voxel_centers[:, 2],
                    c=plt.get_cmap("winter")(norm_vz),
                    s=20,
                    marker="s",
                    alpha=0.6,
                )
    _style_3d_ax(ax4)
    ax5 = fig.add_subplot(235, projection="3d")
    ax5.set_facecolor("#0d0d10")
    ax5.set_title("5. Poisson Surface Mesh", color="#ffffff", pad=10, fontsize=12)
    if len(filtered_points) > 0:
        mesh = reconstruct_surface_mesh(filtered_points)
        if mesh is not None:
            vertices = np.asarray(mesh.vertices)
            triangles = np.asarray(mesh.triangles)
            if len(triangles) > 0:
                ax5.plot_trisurf(
                    vertices[:, 0],
                    vertices[:, 1],
                    vertices[:, 2],
                    triangles=triangles,
                    cmap="viridis",
                    alpha=0.8,
                    linewidth=0.1,
                    edgecolor="#333",
                )
    _style_3d_ax(ax5)
    ax6 = fig.add_subplot(236)
    ax6.set_facecolor("#0d0d10")
    ax6.set_title(
        f"6. Cross Section at Z = {slice_z:.1f} cm",
        color="#ffffff",
        pad=10,
        fontsize=12,
    )
    if len(slice_pts) > 0:
        dists = np.sqrt(slice_pts[:, 0] ** 2 + slice_pts[:, 1] ** 2)
        d_min, d_max = dists.min(), dists.max()
        d_span = d_max - d_min if d_max - d_min > 1e-9 else 1.0
        norm_d = (dists - d_min) / d_span
        colors_s = plt.get_cmap("viridis")(norm_d)
        ax6.scatter(
            slice_pts[:, 0],
            slice_pts[:, 1],
            c=colors_s,
            s=point_size * 2,
            alpha=0.9,
            linewidths=0,
        )
        ax6.text(
            0.02,
            0.95,
            f"{len(slice_pts)} points in slice",
            transform=ax6.transAxes,
            color="#fdcb6e",
            fontsize=9,
            verticalalignment="top",
        )
    else:
        ax6.text(
            0.5,
            0.5,
            "No points in this slice",
            transform=ax6.transAxes,
            color="#666",
            fontsize=12,
            ha="center",
            va="center",
        )
    ax6.set_xlabel("X (cm)", color="#ccc", fontsize=10)
    ax6.set_ylabel("Y (cm)", color="#ccc", fontsize=10)
    ax6.tick_params(colors="#888", labelsize=8)
    ax6.set_aspect("equal", adjustable="datalim")
    ax6.grid(True, color="#1a1a2a", linewidth=0.4)
    for spine in ax6.spines.values():
        spine.set_edgecolor("#333")
    plt.tight_layout(pad=3.0, rect=[0, 0, 1, 0.94])
    fig.savefig(out_png, dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
    print(f"[render] 분석 결과 저장 → {out_png}")
    plt.show()


def _style_3d_ax(ax):
    ax.set_xlabel("X (cm)", color="#ccc", labelpad=8, fontsize=9)
    ax.set_ylabel("Y (cm)", color="#ccc", labelpad=8, fontsize=9)
    ax.set_zlabel("Z (cm)", color="#ccc", labelpad=8, fontsize=9)
    ax.tick_params(colors="#888", labelsize=7)
    for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
        pane.fill = False
        pane.set_edgecolor("#222")
    ax.grid(True, color="#1a1a2a", linewidth=0.3)


def save_filtered_csv(
    path: Path,
    data: dict,
    mask: np.ndarray,
) -> int:
    path.parent.mkdir(parents=True, exist_ok=True)
    count = 0
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            ["yaw_deg", "pitch_deg", "distance_cm", "strength", "x_cm", "y_cm", "z_cm"]
        )
        for i in range(data["n"]):
            if not mask[i]:
                continue
            writer.writerow(
                [
                    data["yaw"][i],
                    data["pitch"][i],
                    data["dist"][i],
                    data["strength"][i],
                    data["x"][i],
                    data["y"][i],
                    data["z"][i],
                ]
            )
            count += 1
    return count


def print_analysis(
    data: dict,
    inlier_mask: np.ndarray,
    bbox: dict,
):
    sep = "═" * 56
    n_total = data["n"]
    n_inlier = int(inlier_mask.sum())
    n_outlier = n_total - n_inlier
    print(f"\n{sep}")
    print("  LiDAR Point Cloud Analysis Report")
    print(sep)
    print(f"  전체 포인트      : {n_total:,}")
    print(f"  정상 (inlier)    : {n_inlier:,}")
    print(f"  이상치 (outlier) : {n_outlier:,}  ({n_outlier/max(n_total,1)*100:.1f}%)")
    print(f"  ──────────────────────────────────────")
    print(f"  바운딩 박스 치수:")
    s = bbox["size"]
    print(f"    가로 (X)  : {s[0]:.2f} cm")
    print(f"    세로 (Y)  : {s[1]:.2f} cm")
    print(f"    높이 (Z)  : {s[2]:.2f} cm")
    print(f"    부피       : {bbox['volume']:.1f} cm³")
    print(f"  ──────────────────────────────────────")
    x, y, z = data["x"][inlier_mask], data["y"][inlier_mask], data["z"][inlier_mask]
    if len(x) > 0:
        print(f"  X 범위  : {x.min():.2f} → {x.max():.2f}  (span {np.ptp(x):.2f} cm)")
        print(f"  Y 범위  : {y.min():.2f} → {y.max():.2f}  (span {np.ptp(y):.2f} cm)")
        print(f"  Z 범위  : {z.min():.2f} → {z.max():.2f}  (span {np.ptp(z):.2f} cm)")
    print(sep + "\n")


def main():
    parser = argparse.ArgumentParser(description="LiDAR 포인트 클라우드 후처리")
    parser.add_argument(
        "csv", nargs="?", default="scan_points.csv", help="입력 CSV 파일"
    )
    parser.add_argument(
        "--k", type=int, default=None, help="SOR k-NN 이웃 수 (기본: 해상도 자동 결정)"
    )
    parser.add_argument(
        "--std-ratio", type=float, default=1.5, help="SOR 표준편차 비율 (기본: 1.5)"
    )
    parser.add_argument(
        "--slice-z", type=float, default=None, help="단면도 높이 (기본: 중간값)"
    )
    parser.add_argument("--output", default=None, help="필터링된 CSV 출력 파일")
    parser.add_argument("--output-png", default=None, help="분석 결과 PNG 파일")
    parser.add_argument(
        "--no-zbuffer", action="store_true", help="z-Buffer 유령 포인트 제거 비활성화"
    )
    parser.add_argument(
        "--no-adaptive-sor",
        action="store_true",
        help="Adaptive SOR 비활성화 (고정 k=8 사용)",
    )
    parser.add_argument(
        "--yaw-bin",
        type=float,
        default=1.0,
        help="z-Buffer yaw 격자 크기 (도, 기본: 1.0)",
    )
    parser.add_argument(
        "--pitch-bin",
        type=float,
        default=0.5,
        help="z-Buffer pitch 격자 크기 (도, 기본: 0.5)",
    )
    args = parser.parse_args()
    csv_path = Path(args.csv)
    print(f"[main] Loading '{csv_path}' ...")
    data = load_csv(csv_path)
    print(f"[main] {data['n']:,} points loaded")
    points = np.column_stack([data["x"], data["y"], data["z"]])
    if not args.no_zbuffer:
        zbuf_mask = zbuffer_filter(
            data, yaw_bin_deg=args.yaw_bin, pitch_bin_deg=args.pitch_bin
        )
        points = points[zbuf_mask]
        for key in ["x", "y", "z", "yaw", "pitch", "dist", "strength"]:
            data[key] = data[key][zbuf_mask]
        data["n"] = int(zbuf_mask.sum())
    else:
        print("[zBuffer] 건너뜀 (--no-zbuffer)")
    if not args.no_adaptive_sor:
        auto_k, auto_voxel = compute_adaptive_sor_params(data)
        k_final = args.k if args.k is not None else auto_k
        voxel_final = auto_voxel
    else:
        k_final = args.k if args.k is not None else 8
        voxel_final = 0.5
        print(f"[AdaptiveSOR] 건너뜀 — 고정 k={k_final}, voxel={voxel_final}")
    print(f"[SOR] k={k_final}, std_ratio={args.std_ratio}, voxel={voxel_final:.2f}")
    inlier_mask, mean_dists = statistical_outlier_removal(
        points,
        k=k_final,
        std_ratio=args.std_ratio,
        voxel_size=voxel_final,
    )
    filtered_points = points[inlier_mask]
    print(
        f"[SOR] {inlier_mask.sum():,} inliers, {(~inlier_mask).sum():,} outliers removed"
    )
    fill_data = {
        "yaw": data["yaw"][inlier_mask],
        "pitch": data["pitch"][inlier_mask],
        "dist": data["dist"][inlier_mask],
        "strength": data["strength"][inlier_mask],
        "x": data["x"][inlier_mask],
        "y": data["y"][inlier_mask],
        "z": data["z"][inlier_mask],
        "n": int(inlier_mask.sum()),
    }
    fill_data, n_filled = spherical_hole_fill(fill_data)
    if n_filled > 0:
        n_original_inliers = int(inlier_mask.sum())
        new_pts = np.column_stack(
            [
                fill_data["x"][n_original_inliers:],
                fill_data["y"][n_original_inliers:],
                fill_data["z"][n_original_inliers:],
            ]
        )
        points = np.vstack([points, new_pts])
        inlier_mask = np.concatenate([inlier_mask, np.ones(n_filled, dtype=bool)])
        filtered_points = points[inlier_mask]
        for key in ["yaw", "pitch", "dist", "strength", "x", "y", "z"]:
            data[key] = np.concatenate([data[key], fill_data[key][n_original_inliers:]])
        data["n"] += n_filled
    if len(filtered_points) > 0:
        bbox = compute_bounding_box(filtered_points)
    else:
        bbox = compute_bounding_box(points)
    print_analysis(data, inlier_mask, bbox)
    if args.slice_z is not None:
        slice_z = args.slice_z
    elif len(filtered_points) > 0:
        z_range = filtered_points[:, 2]
        slice_z = (z_range.min() + z_range.max()) / 2.0
    else:
        slice_z = 0.0
    z_span = np.ptp(filtered_points[:, 2]) if len(filtered_points) > 0 else 1.0
    thickness = max(z_span * 0.1, 0.5)
    slice_pts = cross_section(filtered_points, slice_z, thickness)
    print(f"[Slice] Z={slice_z:.1f} ± {thickness/2:.1f} cm → {len(slice_pts)} points")
    if args.output:
        out_csv_path = Path(args.output)
    else:
        out_csv_path = csv_path.parent / (csv_path.stem + "_filtered.csv")
    n_saved = save_filtered_csv(out_csv_path, data, inlier_mask)
    print(f"[save] 필터링된 CSV 저장 → {out_csv_path} ({n_saved} points)")
    if args.output_png:
        out_png = args.output_png
    else:
        out_png = str(csv_path.parent / (csv_path.stem + "_analysis.png"))
    render_analysis(
        data=data,
        filtered_points=filtered_points,
        original_points=points,
        inlier_mask=inlier_mask,
        bbox=bbox,
        slice_z=slice_z,
        slice_pts=slice_pts,
        out_png=out_png,
    )


if __name__ == "__main__":
    main()
