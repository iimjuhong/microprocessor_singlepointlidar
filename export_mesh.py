
from __future__ import annotations
import argparse
import sys
from pathlib import Path
import numpy as np
try:
    import open3d as o3d
except ImportError:
    print("ERROR: Open3D is required. Install with: pip install open3d")
    sys.exit(1)
def load_filtered_csv(path: str | Path) -> np.ndarray:

    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"CSV 파일 없음: {path}")
    raw = np.genfromtxt(
        path, delimiter=",", names=True,
        dtype=None, encoding="utf-8", autostrip=True,
    )
    headers = list(raw.dtype.names)
    norm = {h.strip().lower(): h for h in headers}
    def find(aliases):
        for a in aliases:
            if a.lower() in norm:
                return raw[norm[a.lower()]].astype(np.float64)
        return None
    x = find(["x_cm", "X_cm"])
    y = find(["y_cm", "Y_cm"])
    z = find(["z_cm", "Z_cm"])
    if x is None or y is None or z is None:
        raise ValueError(f"CSV에 x_cm, y_cm, z_cm 컬럼이 필요합니다. 발견된 헤더: {headers}")
    return np.column_stack([x, y, z])
def reconstruct_and_export(points: np.ndarray, output_path: str, depth: int = 8) -> None:

    print(f"[mesh] {len(points):,} points loaded")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    