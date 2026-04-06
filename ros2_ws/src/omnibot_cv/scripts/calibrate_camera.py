#!/usr/bin/env python3
import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml


def camera_info_yaml(width: int, height: int, k: np.ndarray, d: np.ndarray) -> dict:
    return {
        'image_width': int(width),
        'image_height': int(height),
        'camera_name': 'camera',
        'camera_matrix': {'rows': 3, 'cols': 3, 'data': k.reshape(-1).tolist()},
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': {'rows': 1, 'cols': int(d.size), 'data': d.reshape(-1).tolist()},
        'rectification_matrix': {'rows': 3, 'cols': 3, 'data': np.eye(3).reshape(-1).tolist()},
        'projection_matrix': {
            'rows': 3,
            'cols': 4,
            'data': np.hstack([k, np.zeros((3, 1), dtype=np.float64)]).reshape(-1).tolist(),
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description='Calibrate camera from chessboard frames.')
    parser.add_argument('--images', required=True, help='Glob pattern, e.g. calib_frames/*.png')
    parser.add_argument(
        '--pattern-cols',
        '--corners-x',
        dest='pattern_cols',
        type=int,
        default=9,
        help='Chessboard inner corners along X (columns), default: 9',
    )
    parser.add_argument(
        '--pattern-rows',
        '--corners-y',
        dest='pattern_rows',
        type=int,
        default=6,
        help='Chessboard inner corners along Y (rows), default: 6',
    )
    parser.add_argument('--square-size-m', type=float, default=0.024)
    parser.add_argument('--output', default='camera_calibration.yaml')
    args = parser.parse_args()

    if args.pattern_cols < 2 or args.pattern_rows < 2:
        raise ValueError('pattern size must be at least 2x2 inner corners')

    pattern_size = (args.pattern_cols, args.pattern_rows)
    objp = np.zeros((args.pattern_rows * args.pattern_cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.pattern_cols, 0:args.pattern_rows].T.reshape(-1, 2)
    objp *= args.square_size_m

    obj_points = []
    img_points = []
    img_size = None

    for image_path in sorted(Path('.').glob(args.images)):
        frame = cv2.imread(str(image_path))
        if frame is None:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, pattern_size)
        if not found:
            continue
        cv2.cornerSubPix(
            gray,
            corners,
            winSize=(11, 11),
            zeroZone=(-1, -1),
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
        )
        obj_points.append(objp)
        img_points.append(corners)
        img_size = (gray.shape[1], gray.shape[0])

    if not obj_points or img_size is None:
        raise RuntimeError('No valid chessboard detections found.')

    _, k, d, _, _ = cv2.calibrateCamera(obj_points, img_points, img_size, None, None)

    data = camera_info_yaml(img_size[0], img_size[1], k, d)
    with open(args.output, 'w', encoding='utf-8') as f:
        yaml.safe_dump(data, f, sort_keys=False)

    print(f'Calibration saved to: {args.output}')


if __name__ == '__main__':
    main()
