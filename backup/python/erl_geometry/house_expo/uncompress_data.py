import glob
import os
from typing import List

from .list_data import get_map_dir
from .list_data import get_traj_dir

__all__ = ["uncompress_json_data", "uncompress_traj_data"]

_file_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")


def get_json_tar_file() -> str:
    return os.path.join(_file_dir, "json.tar.gz")


def get_traj_tar_files() -> List[str]:
    return sorted(glob.glob(os.path.join(_file_dir, "traj.tar.gz-*")))


def uncompress_json_data() -> None:
    json_dir = get_map_dir()
    json_tar_file = get_json_tar_file()
    if os.path.exists(json_tar_file):
        print("Uncompress HouseExpoMap json files ...")
        output_dir = os.path.dirname(json_dir)
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        os.system(f"tar -xf {json_tar_file} -C {output_dir}")
    else:
        raise FileNotFoundError(f"{json_tar_file} does not exist.")


def uncompress_traj_data() -> None:
    traj_dir = get_traj_dir()
    output_dir = os.path.dirname(traj_dir)
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    traj_tar_files = get_traj_tar_files()
    traj_tar_file = os.path.join(output_dir, "traj.tar.gz")
    os.system(f"cat {' '.join(traj_tar_files)} > {traj_tar_file}")

    print("Uncompress HouseExpoMap trajectory files ...")
    os.system(f"tar -xf {traj_tar_file} -C {output_dir}")
    os.remove(traj_tar_file)


def main() -> None:
    print("Current directory:", os.path.realpath(os.curdir))
    print("Script path:", __file__)
    uncompress_json_data()
    uncompress_traj_data()


if __name__ == "__main__":
    main()
