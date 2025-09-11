import argparse
import os
from typing import Tuple

import matplotlib.pyplot as plt
import numpy as np
import numpy.typing as npt
from erl_common.storage import GridMapInfo2D

from . import HouseExpoMap


def generate_png(
    json_file: str, meter2pixel: int, padding: int, output: str
) -> Tuple[HouseExpoMap, GridMapInfo2D, npt.NDArray[np.float64]]:
    house_expo_map = HouseExpoMap(json_file)
    space = house_expo_map.meter_space
    vertices = space.surface.vertices
    mins = np.min(vertices, axis=1)
    maxs = np.max(vertices, axis=1)
    grid_map_info = GridMapInfo2D(mins, maxs, np.array([1 / meter2pixel] * 2), np.array([padding] * 2))
    map_image: np.ndarray = space.generate_map_image(grid_map_info)
    plt.imsave(output, map_image, cmap="binary_r")
    return house_expo_map, grid_map_info, map_image


def main() -> None:
    print("Current directory:", os.path.realpath(os.curdir))
    print("Script path:", __file__)

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--json-file",
        default=os.path.join(
            os.path.dirname(__file__),
            "0a1b29dba355df2ab02630133187bfab.json",
        ),
        help="JSON file of a room in the HouseExpo dataset.",
    )
    parser.add_argument(
        "--meter2pixel",
        default=100,
        type=int,
        help="Number of pixels corresponding to one meter.",
    )
    parser.add_argument("--padding", default=2, type=int, help="In pixels.")
    parser.add_argument("--output", default="house_expo.png", type=str, help="Path to save the png.")
    parser.add_argument("--show", action="store_true", help="Show the image after saving it.")
    args = parser.parse_args()

    map_image = generate_png(args.json_file, args.meter2pixel, args.padding, args.output)[-1]

    if args.show:
        plt.imshow(map_image, cmap="binary_r")
        plt.show()


if __name__ == "__main__":
    main()
