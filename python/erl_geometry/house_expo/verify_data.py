import argparse
import os
import pickle
import shutil
import subprocess
from enum import Enum
from typing import Any
from typing import Dict
from typing import List
from typing import Optional

import cv2
import numpy as np
import yaml
from erl_common.storage import GridMapInfo2D
from tqdm import tqdm

from .list_data import get_bad_json_filenames
from .list_data import get_map_dir
from .list_data import get_map_filenames
from .sequence import HouseExpoSequence


class VerfResult(Enum):
    PASS = "PASS"
    LOW_COMPLETION = "LOW_COMPLETION"
    MISSING_FILE = "MISSING_FILE"
    FAILED_TO_FIX = "FAILED_TO_FIX"


def check_trajectory_completion_percentage_of_json_file(json_file: str, path_skip: int, try_to_fix: bool) -> dict:
    data_dir = os.path.dirname(json_file)
    token = os.path.splitext(os.path.basename(json_file))[0]

    path_file = os.path.join(data_dir, f"{token}.csv")

    sequence = HouseExpoSequence(json_file, path_file, lidar_mode="kDdf")
    param_filepath = os.path.join(data_dir, f"{token}.yaml")
    with open(param_filepath, "r") as file:
        params = yaml.full_load(file)

    png_filepath = os.path.join(data_dir, f"{token}.png")
    image_map = cv2.imread(png_filepath)[..., 0]

    vertices = sequence.map.meter_space.surface.vertices
    mins = np.min(vertices, axis=1)
    maxs = np.max(vertices, axis=1)
    grid_map_info = GridMapInfo2D(
        mins, maxs, np.array([1 / params["meter2pixel"]] * 2), np.array([params["padding"]] * 2)
    )

    image_scan = np.zeros_like(image_map)
    rays = grid_map_info.meter_to_pixel_for_points(
        sequence.get_rays_of_frames(range(0, len(sequence), path_skip)).reshape([2, -1], order="F")
    ).T.reshape(
        [-1, 2, 2]
    )  # (4, N) -> (2, 2N) -> (2N, 2) -> (N, 2, 2)
    rays = np.round(rays).astype(np.int32)
    cv2.polylines(image_scan, rays, False, 255, 1)

    intersection = np.sum((image_map > 0) & (image_scan > 0))
    scanned_percentage = intersection / np.sum(image_map > 0)
    tqdm.write(f"{token}: trajectory coverage = {scanned_percentage * 100}")
    required_percentage = params["completion_percentage"]
    if scanned_percentage >= 0.97 * required_percentage:
        return dict(
            verf_result=[VerfResult.PASS],
            info=dict(
                scanned_percentage=scanned_percentage,
                required_percentage=required_percentage,
            ),
        )
    elif try_to_fix:
        log_filename = f"{token}.log"
        render_size_scale = 1000 / max(image_map.shape)
        with open(os.path.join(data_dir, log_filename), "w") as file:
            process = subprocess.run(
                [
                    "erl-geometry-house-expo-generate-trajectory",
                    "--seed",
                    str(params["seed"]),
                    "--json-file",
                    json_file,
                    "--meter2pixel",
                    str(params["meter2pixel"]),
                    "--padding",
                    str(params["padding"]),
                    "--output-dir",
                    data_dir,
                    "--sensor-range",
                    str(params["sensor_max_range"]),
                    "--angular-range",
                    str(params["angular_range"]),
                    "--completion-percentage",
                    "1.0",
                    "--completion-check-interval",
                    "1",
                    "--max-exploration-iterations",
                    str(params["max_exploration_iterations"] * 2),
                    "--render-without-waiting",
                    "--render-size-scale",
                    f"{render_size_scale}"
                    # "--no-render",
                ],
                stdout=file,
                stderr=file,
            )
        try:
            process.check_returncode()
            return check_trajectory_completion_percentage_of_json_file(json_file, path_skip, False)
        except Exception as e:
            print(e)
            return dict(
                verf_result=[VerfResult.LOW_COMPLETION, VerfResult.FAILED_TO_FIX],
                info=dict(
                    scanned_percentage=scanned_percentage,
                    required_percentage=required_percentage,
                ),
            )
    else:
        return dict(
            verf_result=[VerfResult.LOW_COMPLETION],
            info=dict(
                scanned_percentage=scanned_percentage,
                required_percentage=required_percentage,
            ),
        )


def check_data_completeness_of_json_file(
    data_dir: str,
    json_filename: str,
    try_to_fix: bool = True,
    files: Optional[List[str]] = None,
) -> Dict[str, Any]:
    json_file = os.path.join(get_map_dir(), json_filename)
    token = os.path.splitext(json_filename)[0]

    if files is None:
        files = os.listdir(data_dir)

    os.chdir(data_dir)

    csv_filename = f"{token}.csv"
    png_filename = f"{token}.png"
    log_filename = f"{token}.log"
    yaml_filename = f"{token}.yaml"

    if not try_to_fix:
        missing_files = []
        for file in [
            json_filename,
            csv_filename,
            png_filename,
            yaml_filename,
        ]:
            if file not in files:
                missing_files.append(file)
        if len(missing_files) > 0:
            return dict(
                verf_result=[VerfResult.MISSING_FILE],
                info=dict(missing_files=missing_files),
            )
        else:
            return dict(verf_result=[VerfResult.PASS], info=None)

    if yaml_filename not in files:
        return dict(
            verf_result=[VerfResult.MISSING_FILE],
            info=dict(missing_files=[yaml_filename]),
        )

    if json_filename not in files:
        shutil.copyfile(json_file, os.path.join(data_dir, json_filename))

    if csv_filename in files:
        return dict(verf_result=[VerfResult.PASS], info=None)
    else:
        with open(os.path.join(data_dir, yaml_filename), "r") as file:
            params = yaml.full_load(file)

        with open(os.path.join(data_dir, log_filename), "w") as file:
            process = subprocess.run(
                [
                    "gpis-data-house-expo-generate-trajectory",
                    "--seed",
                    str(params["seed"]),
                    "--json-file",
                    json_file,
                    "--meter2pixel",
                    str(params["meter2pixel"]),
                    "--padding",
                    str(params["padding"]),
                    "--output-dir",
                    data_dir,
                    "--sensor-range",
                    str(params["sensor_max_range"]),
                    "--angular-range",
                    str(params["angular_range"]),
                    "--completion-percentage",
                    "1.0",
                    "--completion-check-interval",
                    "1",
                    "--max-exploration-iterations",
                    str(params["max_exploration_iterations"] * 2),
                    "--render-without-waiting",
                    # "--no-render",
                ],
                stdout=file,
                stderr=file,
            )

            try:
                process.check_returncode()
                return dict(verf_result=[VerfResult.PASS], info=None)
            except Exception as e:
                print(e)
                return dict(
                    verf_result=[VerfResult.MISSING_FILE, VerfResult.FAILED_TO_FIX],
                    info=dict(missing_files=[csv_filename]),
                )


def verify_data(
    data_dir: str,
    percent_start: float,
    percent_todo: float,
    path_skip: int,
    try_to_fix: bool = True,
) -> Dict[str, Any]:
    bad_json_files = get_bad_json_filenames()
    json_files = get_map_filenames()
    percent_start = max([min([percent_start, 1.0]), 0.0])
    percent_todo = max([min([percent_todo, 1.0]), 0.0])
    percent_end = max([min([percent_start + percent_todo, 1.0]), 0.0])

    start = int(round(len(json_files) * percent_start))
    end = int(round(len(json_files) * percent_end))

    json_files = [f for f in json_files[start:end] if f not in bad_json_files]

    # try to load the cached progress
    verify_cache_file = os.path.join(data_dir, "verify_cache.pkl")
    verify_cache: Dict[str, Any]
    if os.path.exists(verify_cache_file):
        with open(verify_cache_file, "rb") as file:
            verify_cache = pickle.load(file)
        json_files = [
            f
            for f in json_files
            if (os.path.basename(f) not in verify_cache)
            or (VerfResult.PASS.name not in verify_cache[os.path.basename(f)]["verf_result"])
        ]
    else:
        verify_cache = dict()

    cnt = 0
    for json_file in tqdm(json_files, desc="verify_data", ncols=100, position=1):
        result = check_data_completeness_of_json_file(data_dir, json_file, try_to_fix)
        if VerfResult.PASS in result["verf_result"]:  # pass completeness check
            json_file = os.path.join(data_dir, os.path.basename(json_file))
            result = check_trajectory_completion_percentage_of_json_file(json_file, path_skip, try_to_fix)
        result["verf_result"] = [x.name for x in result["verf_result"]]
        json_file = os.path.basename(json_file)
        verify_cache[json_file] = result

        cnt += 1
        if cnt % 10 == 0:
            with open(verify_cache_file, "wb") as file:
                pickle.dump(verify_cache, file)
    return verify_cache


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-dir", required=True, type=str)
    parser.add_argument("--no-fix", action="store_true")
    parser.add_argument("--percent-start", required=True, type=float)
    parser.add_argument("--percent-todo", required=True, type=float)
    parser.add_argument("--path-skip", default=1, type=int)
    args = parser.parse_args()

    verify_cache = verify_data(
        args.data_dir,
        args.percent_start,
        args.percent_todo,
        args.path_skip,
        not args.no_fix,
    )

    for file, result in verify_cache.items():
        if result["verf_result"] != VerfResult.PASS.name:
            tqdm.write(f"{file}: {result['verf_result']}")


if __name__ == "__main__":
    main()
    # check_trajectory_completion_percentage_of_json_file(
    #     json_file="/home/daizhirui/D/house_expo/e4a38011d30fd1583b4d1e43bbf67ee8.json",
    #     path_skip=1,
    #     try_to_fix=False
    # )
