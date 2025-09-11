import argparse
import os

import cv2
import numpy as np
import yaml
from diff_info_gathering import get_params_filename
from diff_info_gathering.scripts.run_ga_agent import run_ga_agent
from .generate_png import generate_png


def main() -> None:
    print("Current directory:", os.path.realpath(os.curdir))
    print("Script path:", __file__)

    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", default=1, type=int, help="Random seed.")
    parser.add_argument(
        "--json-file",
        type=str,
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
    parser.add_argument(
        "--output-dir",
        default=os.path.abspath(os.curdir),
        help="Directory to save the files.",
    )
    parser.add_argument("--start-state", default=None, nargs="+")
    parser.add_argument("--sensor-range", default=15.0, type=float, help="Lidar radius range.")
    parser.add_argument("--angular-range", default="np.pi * 2", type=str, help="Lidar angular range.")
    parser.add_argument(
        "--angular-resolution",
        default="0.5 * np.pi / 180",
        type=str,
        help="Lidar angular resolution.",
    )
    parser.add_argument("--completion-percentage", default=0.999, type=float)
    parser.add_argument("--completion-check-interval", default=1, type=int)
    parser.add_argument("--max-exploration-iterations", default=500, type=int)
    parser.add_argument("--no-render", action="store_true")
    parser.add_argument("--render-interval", default=1, type=int)
    parser.add_argument("--render-size-scale", default=0.8, type=float)
    parser.add_argument("--render-without-waiting", action="store_true")
    parser.add_argument("--visualize-trajectory", action="store_true")
    args = parser.parse_args()
    args.angular_range = eval(args.angular_range)
    args.angular_resolution = eval(args.angular_resolution)

    token = os.path.splitext(os.path.basename(args.json_file))[0]

    png_filename = f"{token}.png"
    yaml_filename = f"{token}.yaml"
    path_filename = f"{token}.csv"

    os.makedirs(args.output_dir, exist_ok=True)

    args.json_file = os.path.abspath(args.json_file)
    os.chdir(args.output_dir)

    house_expo_map, grid_map_info, map_image = generate_png(
        args.json_file, args.meter2pixel, args.padding, png_filename
    )
    print(
        f"room size: [{grid_map_info.min[0]}, {grid_map_info.max[0]}], [{grid_map_info.min[1]}, {grid_map_info.max[1]}]"
    )

    np.random.seed(args.seed)

    kwargs = args.__dict__.copy()
    del kwargs["seed"]
    del kwargs["json_file"]
    del kwargs["meter2pixel"]
    del kwargs["padding"]
    del kwargs["output_dir"]
    del kwargs["visualize_trajectory"]
    kwargs["map_filename"] = png_filename
    kwargs["params_filename"] = get_params_filename()
    kwargs["map_resolution"] = 1.0 / args.meter2pixel
    kwargs["render"] = not kwargs["no_render"]
    del kwargs["no_render"]
    kwargs["render_wait_for_key"] = not kwargs["render_without_waiting"]
    del kwargs["render_without_waiting"]

    # save the parameters used for generating the path
    params = args.__dict__.copy()
    with open(os.path.join(kwargs["params_filename"])) as f:
        params["diff_info_gathering_params"] = yaml.full_load(f)
    with open(yaml_filename, "w") as f:
        yaml.dump(params, f)

    # get the path
    paths = run_ga_agent(**kwargs)[1]  # [v, u, theta]
    # convert it from pixels to meters
    paths[:, :2] = grid_map_info.pixel_to_meter_for_points(paths[:, :2].T).T  # in meters
    # save the path
    np.savetxt(path_filename, paths, delimiter=",")
    print(f"path saved to: {path_filename}")

    if args.visualize_trajectory:
        import matplotlib.pyplot as plt

        grids = grid_map_info.generate_meter_coordinates(c_stride=True)
        plt.pcolormesh(grids[0], grids[1], map_image, cmap="binary_r")
        plt.plot(paths[:, 0], paths[:, 1])
        plt.show()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
