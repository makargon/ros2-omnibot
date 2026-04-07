#!/usr/bin/env python3
import argparse
from pathlib import Path

import cv2
import yaml


def main() -> None:
    parser = argparse.ArgumentParser(description='Generate ArUco field image and marker map YAML.')
    parser.add_argument('--output-image', default='aruco_field.png')
    parser.add_argument('--output-yaml', default='/tmp/omnibot_aruco_field.yaml')
    parser.add_argument('--dict-name', default='DICT_4X4_50')
    parser.add_argument('--markers-x', type=int, default=4)
    parser.add_argument('--markers-y', type=int, default=3)
    parser.add_argument('--marker-length-px', type=int, default=160)
    parser.add_argument('--marker-separation-px', type=int, default=40)
    parser.add_argument('--marker-length-m', type=float, default=0.06)
    parser.add_argument('--separation-m', type=float, default=0.02)
    args = parser.parse_args()

    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, args.dict_name))
    board = cv2.aruco.GridBoard(
        (args.markers_x, args.markers_y),
        args.marker_length_m,
        args.separation_m,
        dictionary,
    )

    width = args.markers_x * args.marker_length_px + (args.markers_x - 1) * args.marker_separation_px
    height = args.markers_y * args.marker_length_px + (args.markers_y - 1) * args.marker_separation_px
    image = board.generateImage((width, height), marginSize=20, borderBits=1)

    Path(args.output_image).parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(args.output_image, image)

    ids = []
    xs = []
    ys = []
    yaws = []
    step = args.marker_length_m + args.separation_m
    marker_id = 0
    for y in range(args.markers_y):
        for x in range(args.markers_x):
            ids.append(marker_id)
            xs.append(x * step)
            ys.append(y * step)
            yaws.append(0.0)
            marker_id += 1

    data = {
        'aruco_pose_node': {
            'ros__parameters': {
                'field_frame': 'aruco_field',
                'aruco_dictionary': args.dict_name,
                'marker_length_m': args.marker_length_m,
                'marker_ids': ids,
                'marker_x_m': xs,
                'marker_y_m': ys,
                'marker_yaw_rad': yaws,
            }
        }
    }

    Path(args.output_yaml).parent.mkdir(parents=True, exist_ok=True)
    with open(args.output_yaml, 'w', encoding='utf-8') as f:
        yaml.safe_dump(data, f, sort_keys=False)

    print(f'Field image: {args.output_image}')
    print(f'Field map: {args.output_yaml}')


if __name__ == '__main__':
    main()
