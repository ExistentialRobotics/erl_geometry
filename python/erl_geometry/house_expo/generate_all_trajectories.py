import argparse
import os
import shutil
import subprocess
from multiprocessing import Process
from typing import List

from tqdm import tqdm

from .generate_png import generate_png
from .list_data import get_bad_json_filenames
from .list_data import get_map_dir
from .list_data import get_map_filenames
from .verify_data import VerfResult
from .verify_data import check_data_completeness_of_json_file


def worker_func(json_files: List[str], kwargs: dict) -> None:
    output_dir = os.path.abspath(os.curdir)

    if kwargs["reverse"]:
        json_files = list(reversed(json_files))

    tqdm.write(f"Worker {os.getpid()} gets {len(json_files)} to process.")

    bad_json_files = get_bad_json_filenames()

    for json_file in json_files:
        if json_file in bad_json_files:
            tqdm.write(f"{json_file} gives a bad room layout, skip it.")

        token = os.path.splitext(os.path.basename(json_file))[0]
        json_filename = os.path.basename(json_file)
        png_filename = f"{token}.png"
        csv_filename = f"{token}.csv"
        yaml_filename = f"{token}.yaml"
        log_filename = f"{token}.log"

        tqdm.write(json_file)
        missing_files = []
        for file in [
            json_filename,
            png_filename,
            csv_filename,
            yaml_filename,
        ]:
            if not os.path.exists(file):
                missing_files.append(file)
        if len(missing_files) == 0:
            continue
        elif len(missing_files) > 0 and not kwargs["pick_unfinished"]:
            tqdm.write(f"skip unfinished {json_file}")
            continue
        else:
            shutil.copyfile(json_file, json_filename)

            generate_png(json_file, kwargs["meter2pixel"], kwargs["padding"], png_filename)

            with open(log_filename, "w") as file:
                process = subprocess.run(
                    [
                        "erl-geometry-house-expo-generate-trajectory",
                        "--seed",
                        str(kwargs["seed"]),
                        "--json-file",
                        json_file,
                        "--meter2pixel",
                        str(kwargs["meter2pixel"]),
                        "--padding",
                        str(kwargs["padding"]),
                        "--output-dir",
                        output_dir,
                        "--sensor-range",
                        str(kwargs["sensor_max_range"]),
                        "--angular-range",
                        str(kwargs["angular_range"]),
                        "--completion-percentage",
                        str(kwargs["completion_percentage"]),
                        "--completion-check-interval",
                        str(kwargs["completion_check_interval"]),
                        "--max-exploration-iterations",
                        str(kwargs["max_exploration_iterations"]),
                        "--no-render",
                    ],
                    stdout=file,
                    stderr=file,
                )
            try:
                process.check_returncode()  # deal with zombie process
            except subprocess.CalledProcessError:
                tqdm.write(f"[ERROR] occurs when processing {json_file}")

            break


def main() -> None:
    print("Current directory:", os.path.realpath(os.curdir))
    print("Script path:", __file__)

    parser = argparse.ArgumentParser()
    parser.add_argument("--num-workers", default=os.cpu_count(), type=int)
    parser.add_argument("--seed", default=1, type=int, help="Random seed.")
    parser.add_argument(
        "--percent-start",
        required=True,
        type=float,
        help="between 0 and 1, percentage of json files to skip",
    )
    parser.add_argument(
        "--percent-todo",
        required=True,
        type=float,
        help="between 0 and 1, percentage of json files to process",
    )
    parser.add_argument(
        "--meter2pixel",
        default=100,
        type=int,
        help="Number of pixels corresponding to one meter.",
    )
    parser.add_argument("--padding", default=2, type=int, help="In pixels.")
    parser.add_argument("--output-dir", required=True, help="Directory to save the files.")
    parser.add_argument("--start-state", default=None, nargs="+")
    parser.add_argument("--sensor-range", default=15.0, type=float, help="Lidar radius range.")
    parser.add_argument("--angular-range", default="np.pi * 2", type=str, help="Lidar angular range.")
    parser.add_argument(
        "--angular-resolution",
        default="0.5 * np.pi / 180",
        type=str,
        help="Lidar angular resolution.",
    )
    parser.add_argument("--completion-percentage", default=1.0, type=float)
    parser.add_argument("--completion-check-interval", default=1, type=int)
    parser.add_argument("--max-exploration-iterations", default=50, type=int)
    parser.add_argument("--reverse", action="store_true")
    parser.add_argument("--pick-unfinished", action="store_true")

    args = parser.parse_args()

    percent_start = max([min([args.percent_start, 1.0]), 0.0])
    percent_todo = max([min([args.percent_todo, 1.0]), 0.0])
    percent_end = max([min([percent_start + percent_todo, 1.0]), 0.0])
    num_workers = args.num_workers
    args.output_dir = os.path.abspath(args.output_dir)

    if not os.path.isdir(args.output_dir):
        os.makedirs(args.output_dir, exist_ok=True)

    json_dir = get_map_dir()
    json_files = get_map_filenames()

    start = int(round(len(json_files) * percent_start))
    end = int(round(len(json_files) * percent_end))

    files = os.listdir(args.output_dir)  # optimization of disk IO
    json_files = [
        os.path.join(json_dir, f)
        for f in json_files[start:end]
        if VerfResult.PASS
        not in check_data_completeness_of_json_file(args.output_dir, f, try_to_fix=False, files=files)["verf_result"]
    ]
    tqdm.write(f"{len(json_files)} json files to compute.")
    for f in json_files:
        tqdm.write(os.path.basename(f))

    n = len(json_files)
    if n == 0:
        return

    num_workers = min([num_workers, n])
    batch_size = n // num_workers
    leftover = n - batch_size * num_workers

    os.chdir(args.output_dir)
    workers = []
    s2 = 0
    for i in range(num_workers):
        s1 = s2
        s2 = s1 + batch_size + int(leftover > i)
        worker = Process(target=worker_func, args=[json_files[s1:s2], args.__dict__])
        workers.append(worker)
        worker.start()

    for worker in workers:
        worker.join()


if __name__ == "__main__":
    main()
