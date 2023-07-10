import os
from typing import List

__all__ = [
    "get_map_dir",
    "get_map_filenames",
    "get_bad_json_filenames",
    "get_traj_dir",
    "get_map_and_traj_files",
]

_file_dir = os.path.dirname(os.path.abspath(__file__))


def get_map_dir() -> str:
    return os.path.join(os.environ["HOME"], ".cache", "erl_geometry", "house_expo", "data", "json")


def get_map_filenames() -> List[str]:
    json_dir = get_map_dir()
    return sorted(os.listdir(json_dir))


def get_bad_json_filenames() -> List[str]:
    """Return a list of json files that define bad room layouts in the original HouseExpo dataset.

    Returns:
        List[str]: list of bad json filenames.
    """
    with open(os.path.join(_file_dir, "bad_json_files.txt"), "r") as file:
        lines = file.readlines()
    return [line for line in lines if len(line) > 0]


def get_traj_dir() -> str:
    return os.path.join(os.environ["HOME"], ".cache", "erl_geometry", "house_expo", "data", "traj")


def get_map_and_traj_files() -> List[List[str]]:
    json_dir = get_map_dir()
    traj_dir = get_traj_dir()
    files = []
    for csv_file in sorted(os.listdir(traj_dir)):
        token = os.path.splitext(csv_file)[0]
        files.append([os.path.join(json_dir, f"{token}.json"), os.path.join(traj_dir, csv_file)])
    return files


def main() -> None:
    json_filenames = get_map_filenames()
    print(f"{len(json_filenames)} files in total.")
    json_traj_files = get_map_and_traj_files()
    print(f"{len(json_traj_files)} pairs of room and trajectory in total.")
    print("Current directory:", os.path.realpath(os.curdir))
    print("Script path:", __file__)
    print("HouseExpo JSON folder:", get_map_dir())
    print("HouseExpo Trajectory folder:", get_traj_dir())


if __name__ == "__main__":
    main()
