import argparse
import glob
import os
import pickle
import shutil

from tqdm import tqdm

from .verify_data import VerfResult


def main() -> None:
    print("Current directory:", os.path.realpath(os.curdir))
    print("Script path:", __file__)
    parser = argparse.ArgumentParser()
    parser.add_argument("--verify-result-pkl", required=True, type=str)
    parser.add_argument("--input-dir", type=str)
    parser.add_argument("--output-dir", required=True, type=str)

    args = parser.parse_args()
    if args.input_dir is None:
        args.input_dir = os.path.dirname(args.verify_result_pkl)
    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir, exist_ok=True)

    with open(args.verify_result_pkl, "rb") as file:
        results: dict = pickle.load(file)

    json_files = glob.glob(os.path.join(args.input_dir, "**/*.json"))
    json_name_to_path = {os.path.basename(f): f for f in json_files}

    cnt = 0
    for json_name, result in tqdm(results.items(), ncols=80):
        if VerfResult.PASS.name not in result["verf_result"]:
            continue
        json_file_src = json_name_to_path[json_name]
        token = os.path.splitext(json_name)[0]
        csv_name = f"{token}.csv"
        csv_file_src = os.path.join(os.path.dirname(json_file_src), csv_name)

        shutil.copyfile(csv_file_src, os.path.join(args.output_dir, csv_name))
        cnt += 1

    tqdm.write(f"{cnt} trajectories are collected and stored at {args.output_dir}.")


if __name__ == "__main__":
    main()
