#!/usr/bin/env python3

import json
from pathlib import Path

import cv2
import numpy as np


def save_opencv_yaml(path, data):
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    if not fs.isOpened():
        raise RuntimeError(f"Failed to open {path} for writing")

    for key, value in data.items():
        if isinstance(value, str):
            fs.write(key, value)
        elif isinstance(value, (int, float)):
            fs.write(key, value)
        else:
            fs.write(key, np.asarray(value))
    fs.release()


def main():
    home = Path.home()
    pairs_dir = home / "sondre_bot_gz" / "stereo_pairs"
    left_dir = pairs_dir / "left"
    right_dir = pairs_dir / "right"
    out_dir = home / "sondre_bot_gz" / "config"
    out_dir.mkdir(parents=True, exist_ok=True)

    pattern_size = (7, 10)   # inner corners
    square_size_m = 0.024    # 24 mm

    left_images = sorted(left_dir.glob("left_*.png"))
    right_images = sorted(right_dir.glob("right_*.png"))

    if not left_images or not right_images:
        raise RuntimeError("No stereo images found")

    right_map = {p.name.replace("right_", ""): p for p in right_images}
    pairs = []
    for lp in left_images:
        suffix = lp.name.replace("left_", "")
        rp = right_map.get(suffix)
        if rp is not None:
            pairs.append((lp, rp))

    if len(pairs) < 10:
        raise RuntimeError(f"Too few stereo pairs found: {len(pairs)}")

    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size_m

    objpoints = []
    imgpoints_left = []
    imgpoints_right = []

    image_size = None
    accepted = []

    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        100,
        1e-5,
    )

    for left_path, right_path in pairs:
        left = cv2.imread(str(left_path), cv2.IMREAD_GRAYSCALE)
        right = cv2.imread(str(right_path), cv2.IMREAD_GRAYSCALE)

        if left is None or right is None:
            continue

        if image_size is None:
            image_size = (left.shape[1], left.shape[0])

        if hasattr(cv2, "findChessboardCornersSB"):
            found_l, corners_l = cv2.findChessboardCornersSB(left, pattern_size, None)
            found_r, corners_r = cv2.findChessboardCornersSB(right, pattern_size, None)
        else:
            flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
            found_l, corners_l = cv2.findChessboardCorners(left, pattern_size, flags)
            found_r, corners_r = cv2.findChessboardCorners(right, pattern_size, flags)

            if found_l:
                cv2.cornerSubPix(left, corners_l, (11, 11), (-1, -1), criteria)
            if found_r:
                cv2.cornerSubPix(right, corners_r, (11, 11), (-1, -1), criteria)

        if found_l and found_r:
            objpoints.append(objp.copy())
            imgpoints_left.append(corners_l.astype(np.float32))
            imgpoints_right.append(corners_r.astype(np.float32))
            accepted.append((left_path.name, right_path.name))

    if len(objpoints) < 10:
        raise RuntimeError(f"Too few valid checkerboard pairs: {len(objpoints)}")

    print(f"Using {len(objpoints)} valid stereo pairs")

    ret_l, K1, D1, _, _ = cv2.calibrateCamera(
        objpoints,
        imgpoints_left,
        image_size,
        None,
        None,
    )

    ret_r, K2, D2, _, _ = cv2.calibrateCamera(
        objpoints,
        imgpoints_right,
        image_size,
        None,
        None,
    )

    stereo_flags = cv2.CALIB_FIX_INTRINSIC

    ret_s, K1, D1, K2, D2, R, T, E, F = cv2.stereoCalibrate(
        objpoints,
        imgpoints_left,
        imgpoints_right,
        K1,
        D1,
        K2,
        D2,
        image_size,
        criteria=criteria,
        flags=stereo_flags,
    )

    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        K1,
        D1,
        K2,
        D2,
        image_size,
        R,
        T,
        flags=cv2.CALIB_ZERO_DISPARITY,
        alpha=0.0,
    )

    print(f"Left RMS:   {ret_l:.6f}")
    print(f"Right RMS:  {ret_r:.6f}")
    print(f"Stereo RMS: {ret_s:.6f}")
    print(f"Image size: {image_size}")

    left_yaml = out_dir / "zed_left_camera.yaml"
    right_yaml = out_dir / "zed_right_camera.yaml"
    stereo_yaml = out_dir / "zed_stereo.yaml"
    accepted_json = out_dir / "zed_stereo_pairs_used.json"

    # ROS-style mono camera YAMLs
    left_data = {
        "image_width": int(image_size[0]),
        "image_height": int(image_size[1]),
        "camera_name": "left_camera",
        "camera_matrix": np.array(K1, dtype=np.float64),
        "distortion_model": "plumb_bob",
        "distortion_coefficients": np.array(D1.reshape(-1), dtype=np.float64),
        "rectification_matrix": np.array(R1, dtype=np.float64),
        "projection_matrix": np.array(P1, dtype=np.float64),
    }

    right_data = {
        "image_width": int(image_size[0]),
        "image_height": int(image_size[1]),
        "camera_name": "right_camera",
        "camera_matrix": np.array(K2, dtype=np.float64),
        "distortion_model": "plumb_bob",
        "distortion_coefficients": np.array(D2.reshape(-1), dtype=np.float64),
        "rectification_matrix": np.array(R2, dtype=np.float64),
        "projection_matrix": np.array(P2, dtype=np.float64),
    }

    # OpenCV FileStorage YAMLs for direct loading in code
    save_opencv_yaml(left_yaml, left_data)
    save_opencv_yaml(right_yaml, right_data)

    stereo_data = {
        "image_width": int(image_size[0]),
        "image_height": int(image_size[1]),
        "K1": K1,
        "D1": D1.reshape(-1),
        "K2": K2,
        "D2": D2.reshape(-1),
        "R": R,
        "T": T.reshape(-1),
        "E": E,
        "F": F,
        "R1": R1,
        "R2": R2,
        "P1": P1,
        "P2": P2,
        "Q": Q,
        "stereo_rms": float(ret_s),
        "square_size_m": float(square_size_m),
        "board_cols_inner": int(pattern_size[0]),
        "board_rows_inner": int(pattern_size[1]),
    }
    save_opencv_yaml(stereo_yaml, stereo_data)

    with open(accepted_json, "w") as f:
        json.dump(
            {
                "used_pairs": accepted,
                "count": len(accepted),
                "pattern_size_inner": list(pattern_size),
                "square_size_m": square_size_m,
                "left_rms": float(ret_l),
                "right_rms": float(ret_r),
                "stereo_rms": float(ret_s),
            },
            f,
            indent=2,
        )

    print(f"Saved:")
    print(f"  {left_yaml}")
    print(f"  {right_yaml}")
    print(f"  {stereo_yaml}")
    print(f"  {accepted_json}")


if __name__ == "__main__":
    main()