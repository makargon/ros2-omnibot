import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml


def parse_args():
    parser = argparse.ArgumentParser(description='Calibrate fisheye camera from chessboard images.')
    parser.add_argument('--input_dir', required=True, help='Directory with captured chessboard images.')
    parser.add_argument('--output', default='calibration.yaml', help='Output YAML file path.')
    parser.add_argument('--board_cols', type=int, default=9, help='Inner corners per chessboard row.')
    parser.add_argument('--board_rows', type=int, default=6, help='Inner corners per chessboard column.')
    parser.add_argument('--square_size', type=float, default=0.025, help='Chessboard square size in meters.')
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    pattern_size = (args.board_cols, args.board_rows)

    objp = np.zeros((1, args.board_cols * args.board_rows, 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:args.board_cols, 0:args.board_rows].T.reshape(-1, 2)
    objp *= args.square_size

    obj_points = []
    img_points = []
    image_size = None

    image_paths = sorted(Path(args.input_dir).glob('*.png')) + sorted(Path(args.input_dir).glob('*.jpg'))
    if not image_paths:
        raise RuntimeError(f'No images found in {args.input_dir}')

    for image_path in image_paths:
        image = cv2.imread(str(image_path))
        if image is None:
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, pattern_size)
        if not found:
            continue

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
        cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1), criteria)

        obj_points.append(objp)
        img_points.append(corners)
        image_size = gray.shape[::-1]

    if len(obj_points) < 8:
        raise RuntimeError('Not enough valid calibration frames. Need at least 8 with detected chessboard.')

    k = np.zeros((3, 3))
    d = np.zeros((4, 1))
    rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(len(obj_points))]
    tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(len(obj_points))]

    rms, _, _, _, _ = cv2.fisheye.calibrate(
        obj_points,
        img_points,
        image_size,
        k,
        d,
        rvecs,
        tvecs,
        cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_CHECK_COND + cv2.fisheye.CALIB_FIX_SKEW,
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6),
    )

    output = {
        'camera_matrix': {
            'rows': 3,
            'cols': 3,
            'data': [float(x) for x in k.reshape(-1)],
        },
        'distortion_model': 'fisheye',
        'distortion_coefficients': {
            'rows': 4,
            'cols': 1,
            'data': [float(x) for x in d.reshape(-1)],
        },
        'image_width': int(image_size[0]),
        'image_height': int(image_size[1]),
        'rms_error': float(rms),
        'used_images': len(obj_points),
    }

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open('w', encoding='utf-8') as file_handle:
        yaml.safe_dump(output, file_handle, sort_keys=False)

    print(f'Calibration saved to: {output_path}')
    print(f'RMS error: {rms:.6f} | frames used: {len(obj_points)}')


if __name__ == '__main__':
    main()
