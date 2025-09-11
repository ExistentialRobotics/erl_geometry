import argparse

import matplotlib.pyplot as plt
import numpy as np
from tqdm import tqdm

from . import HouseExpoMap
from .list_data import get_map_and_traj_files


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-csv", type=str, default="map_size_statistics.csv")
    parser.add_argument("--output-png", type=str, default="map_size_statistics.png")
    parser.add_argument("--size-step", type=float, default=100)
    args = parser.parse_args()

    size_to_indices = {}
    map_and_traj_files = get_map_and_traj_files()
    for i, (map_file, traj_file) in enumerate(tqdm(map_and_traj_files, ncols=80)):
        house_map = HouseExpoMap(map_file)
        vertices = house_map.meter_space.surface.vertices
        min_x = np.min(vertices[0])
        max_x = np.max(vertices[0])
        min_y = np.min(vertices[1])
        max_y = np.max(vertices[1])
        size = (max_x - min_x) * (max_y - min_y)
        size = int(size / args.size_step) * args.size_step
        if size not in size_to_indices:
            size_to_indices[size] = []
        size_to_indices[size].append(i)

    with open(args.output_csv, "w") as f:
        for size, indices in size_to_indices.items():
            f.write(f"{size}: {indices}\n")

    sizes = list(size_to_indices.keys())
    sizes.sort()
    num_indices = [len(size_to_indices[size]) for size in sizes]
    plt.plot(sizes, num_indices)
    plt.xlabel("Map size (m^2)")
    plt.ylabel("Number of maps")
    plt.tight_layout()
    plt.savefig(args.output_png)
    plt.show()


if __name__ == "__main__":
    main()
