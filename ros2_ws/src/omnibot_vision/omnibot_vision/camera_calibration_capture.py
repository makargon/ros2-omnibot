import argparse
from pathlib import Path

import cv2


def parse_args():
    parser = argparse.ArgumentParser(description='Capture calibration images from USB camera.')
    parser.add_argument('--device', default='0', help='Camera device index or path (default: 0).')
    parser.add_argument('--output_dir', default='calib_images', help='Directory to save captured images.')
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    device = int(args.device) if str(args.device).isdigit() else args.device

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(args.width))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(args.height))

    if not cap.isOpened():
        raise RuntimeError(f'Cannot open camera device: {device}')

    print('SPACE: save frame | Q: quit')
    image_index = 0

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        preview = frame.copy()
        cv2.putText(preview, f'Saved: {image_index}', (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        cv2.imshow('Calibration Capture', preview)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), ord('Q')):
            break
        if key == ord(' '):
            output_file = output_dir / f'calib_{image_index:03d}.png'
            cv2.imwrite(str(output_file), frame)
            print(f'Saved: {output_file}')
            image_index += 1

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
