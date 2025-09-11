import argparse
import os

from .list_data import get_map_and_traj_files


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--json-file", type=str, required=True)
    args = parser.parse_args()

    map_traj_pairs = get_map_and_traj_files()
    map_files = [os.path.basename(x[0]) for x in map_traj_pairs]
    filename = os.path.basename(args.json_file)
    print(map_files.index(filename))


if __name__ == "__main__":
    main()
