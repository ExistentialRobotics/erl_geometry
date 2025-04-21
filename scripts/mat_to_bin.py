import scipy.io.matlab as matlab
import os
import numpy as np
import argparse


def mat_to_bin(mat_file_path):
    data = matlab.loadmat(mat_file_path)
    folder = os.path.dirname(mat_file_path)
    basename = os.path.basename(mat_file_path)
    filename = os.path.splitext(basename)[0]
    for key, value in data.items():
        if key.startswith("__") or key == "version":
            continue
        dat_file = os.path.join(folder, f"{filename}_{key}.dat")
        print(f"Converting {key} to {dat_file} ... ", end="")
        with open(dat_file, "wb") as f:
            f.write(np.array(value.size).astype(np.int64).tobytes())
            f.write(np.array(value.shape[::-1]).astype(np.int64).tobytes())  # for Eigen default column major
            f.write(value.tobytes("C"))
        print(f"Done.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mat-file", type=str, required=True, help="Path to the .mat file")
    args = parser.parse_args()
    mat_file_path = args.mat_file
    if not os.path.exists(mat_file_path):
        raise FileNotFoundError(f"File {mat_file_path} does not exist.")
    mat_to_bin(mat_file_path)
    print(f"Converted {mat_file_path} to binary files.")


if __name__ == "__main__":
    main()
