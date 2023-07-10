import argparse
import os

import cv2

from .list_data import get_map_and_traj_files
from .visualizer import HouseExpoSequenceVisualizer


def main() -> None:
    print("Current directory:", os.path.realpath(os.curdir))
    print("Script path:", __file__)

    default_map_file, default_traj_file = get_map_and_traj_files()[0]

    parser = argparse.ArgumentParser()
    parser.add_argument("--map-file", type=str, default=default_map_file)
    parser.add_argument("--meter2pixel", default=100, type=int)
    parser.add_argument("--padding", default=2, type=int)
    parser.add_argument("--path-file", type=str, default=default_traj_file)
    parser.add_argument("--lidar-mode", type=str, default="kSddfV2", help="kDdf, kSddfV1 or kSddfV2")
    parser.add_argument("--path-skip", default=1, type=int)
    parser.add_argument("--draw-trajectory", action="store_true")
    args = parser.parse_args()

    visualizer = HouseExpoSequenceVisualizer(**args.__dict__)

    max_windows_size = 1080
    for img in visualizer:
        r = max(img.shape) / max_windows_size
        if r > 1:
            r = 1 / r
            img = cv2.resize(img, dsize=None, fx=r, fy=r)
        cv2.imshow("play_trajectory", img)
        cv2.waitKey(1)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
