#!/usr/bin/env python3
import argparse
import os
from pathlib import Path

import cv2


def main() -> None:
    parser = argparse.ArgumentParser(description='Capture chessboard frames for camera calibration.')
    parser.add_argument('--device', default='/dev/video0', help='Video device path or camera index')
    parser.add_argument('--out-dir', default='calib_frames', help='Directory for saved frames')

    parser.add_argument('--image-width', type=int, default=640, help='Requested frame width, default: 640')
    parser.add_argument('--image-height', type=int, default=480, help='Requested frame height, default: 480')
    parser.add_argument(
        '--pixel-format',
        default='YUYV',
        help='Requested pixel format FOURCC (e.g. YUYV, MJPG), default: YUYV',
    )
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
    parser.add_argument('--no-gui', action='store_true', help='Headless mode: do not open OpenCV window')
    parser.add_argument('--max-saved', type=int, default=40, help='Stop after saving this many frames in --no-gui mode')
    parser.add_argument('--min-frame-gap', type=int, default=10, help='Minimal frame gap between auto-saves in --no-gui mode')
    parser.add_argument(
        '--backend',
        choices=['auto', 'v4l2', 'any'],
        default='auto',
        help='Camera backend: auto (default), v4l2 (Linux /dev/video*), or any',
    )
    args = parser.parse_args()

    if args.pattern_cols < 2 or args.pattern_rows < 2:
        raise ValueError('pattern size must be at least 2x2 inner corners')

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    source = int(args.device) if args.device.isdigit() else args.device

    backend_candidates = []
    if args.backend == 'v4l2':
        backend_candidates = [cv2.CAP_V4L2]
    elif args.backend == 'any':
        backend_candidates = [cv2.CAP_ANY]
    else:
        # auto: for Linux camera devices prefer V4L2 to avoid noisy GStreamer errors in Docker.
        if isinstance(source, str) and source.startswith('/dev/video'):
            backend_candidates = [cv2.CAP_V4L2, cv2.CAP_ANY]
        else:
            backend_candidates = [cv2.CAP_ANY]

    cap = None
    for backend in backend_candidates:
        candidate = cv2.VideoCapture(source, backend)
        if candidate.isOpened():
            cap = candidate
            break
        candidate.release()

    if cap is None:
        # last attempt without explicit backend for compatibility
        cap = cv2.VideoCapture(source)

    if not cap.isOpened():
        raise RuntimeError(f'Cannot open camera: {args.device}')

    if args.image_width > 0:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(args.image_width))
    if args.image_height > 0:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(args.image_height))

    pixel_format = (args.pixel_format or '').strip().upper()
    if len(pixel_format) == 4:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*pixel_format))
    elif pixel_format:
        print(f"Warning: invalid --pixel-format '{args.pixel_format}', expected 4 chars (FOURCC).")

    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fourcc_int = int(cap.get(cv2.CAP_PROP_FOURCC))
    actual_fourcc = ''.join(chr((actual_fourcc_int >> (8 * i)) & 0xFF) for i in range(4)).strip('\x00')
    print(
        f'Requested stream: {args.image_width}x{args.image_height}, pixel_format={pixel_format or "(default)"}'
    )
    print(
        f'Negotiated stream: {actual_width}x{actual_height}, pixel_format={actual_fourcc or "unknown"}'
    )

    pattern_size = (args.pattern_cols, args.pattern_rows)
    saved = 0
    frame_idx = 0
    last_saved_frame = -10**9
    has_display = bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
    gui_enabled = (not args.no_gui) and has_display

    print(f'Pattern (inner corners): {args.pattern_cols}x{args.pattern_rows}')
    if gui_enabled:
        print('[q] quit | [s] save frame with detected corners')
    else:
        if not args.no_gui and not has_display:
            print('Display is unavailable; switching to --no-gui mode automatically.')
        print('Headless capture: frames are auto-saved when corners are detected. Press Ctrl+C to stop.')

    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        frame_idx += 1

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, pattern_size)

        if gui_enabled:
            vis = frame.copy()
            if found:
                cv2.drawChessboardCorners(vis, pattern_size, corners, found)
            cv2.imshow('calibration_capture', vis)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            should_save = key == ord('s') and found
        else:
            should_save = found and (frame_idx - last_saved_frame >= args.min_frame_gap)

        if should_save:
            file_path = out_dir / f'frame_{saved:04d}.png'
            cv2.imwrite(str(file_path), frame)
            saved += 1
            last_saved_frame = frame_idx
            print(f'saved: {file_path}')

            if not gui_enabled and saved >= args.max_saved:
                print(f'Saved {saved} frames; stopping (max-saved reached).')
                break

    cap.release()
    if gui_enabled:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
