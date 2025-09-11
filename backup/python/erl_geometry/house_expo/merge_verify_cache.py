import argparse
import glob
import os
import pickle


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-dir", required=True, type=str)
    args = parser.parse_args()

    files = glob.glob(f"{args.data_dir}/**/verify_cache.pkl", recursive=True)
    verify_result = dict()
    for file in files:
        with open(file, "rb") as f:
            verify_result.update(pickle.load(f))
    with open(os.path.join(args.data_dir, "verify_result.pkl"), "wb") as f:
        pickle.dump(verify_result, f)


if __name__ == "__main__":
    main()
