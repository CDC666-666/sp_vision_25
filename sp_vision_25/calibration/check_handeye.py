"""Check a fixed board's world-position consistency using existing calibration parameters.

Usage: python3 calibration/check_handeye.py assets/handeye_run02 -c configs/handeye_standard3.yaml
This does not fit parameters. Low scatter alone does not establish absolute accuracy.
"""

import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml


def quaternion_matrix(q):
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-6:
        raise ValueError("Invalid wxyz quaternion")
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_folder", type=Path)
    parser.add_argument("-c", "--config", type=Path, required=True)
    args = parser.parse_args()
    cfg = yaml.safe_load(args.config.read_text())
    cols, rows = cfg["pattern_cols"], cfg["pattern_rows"]
    obj = np.array([[j, i, 0] for i in range(rows) for j in range(cols)], np.float32)
    obj *= cfg["center_distance_mm"]
    k = np.array(cfg["camera_matrix"], dtype=float).reshape(3, 3)
    d = np.array(cfg["distort_coeffs"], dtype=float)
    m = np.array(cfg["R_gimbal2imubody"], dtype=float).reshape(3, 3)
    r = np.array(cfg["R_camera2gimbal"], dtype=float).reshape(3, 3)
    t = np.array(cfg["t_camera2gimbal"], dtype=float).reshape(3, 1)
    pattern_type = cfg.get("pattern_type", "circles")
    if pattern_type not in ("chessboard", "circles"):
        raise ValueError("pattern_type must be chessboard or circles")
    images = sorted(args.input_folder.glob("[0-9]*.jpg"), key=lambda p: int(p.stem))
    positions, errors, ids = [], [], []
    for path in images:
        gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            raise ValueError(f"Cannot read {path}")
        if pattern_type == "chessboard":
            ok, points = cv2.findChessboardCorners(
                gray, (cols, rows), cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
            )
            if ok:
                cv2.cornerSubPix(gray, points, (11, 11), (-1, -1), (3, 30, 0.001))
        else:
            ok, points = cv2.findCirclesGrid(gray, (cols, rows))
        if not ok:
            print(f"Skipped: {path}")
            continue
        q = np.loadtxt(path.with_suffix(".txt"))
        g = m.T @ quaternion_matrix(q) @ m
        ok, rv, tv = cv2.solvePnP(obj, points, k, d, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            raise ValueError(f"PnP failed: {path}")
        reprojection = cv2.projectPoints(obj, rv, tv, k, d)[0]
        errors.append(np.linalg.norm(reprojection - points, axis=2).mean())
        positions.append((g @ (r @ (tv / 1000) + t)).ravel())
        ids.append(path.stem)
    if len(positions) < 3:
        raise ValueError("At least 3 valid image/quaternion pairs are required")
    positions = np.array(positions)
    deviations = np.linalg.norm(positions - positions.mean(axis=0), axis=1) * 1000
    print(f"有效图片: {len(positions)}/{len(images)}")
    print(f"平均PnP重投影误差: {np.mean(errors):.4f} px")
    print(f"固定棋盘位置相对均值的RMS偏差: {np.sqrt(np.mean(deviations**2)):.2f} mm")
    print(f"最大偏离均值: {deviations.max():.2f} mm (图片 {ids[int(deviations.argmax())]})")
    print("该检查衡量一致性，不代表平移参数的绝对测量误差。")


if __name__ == "__main__":
    main()
