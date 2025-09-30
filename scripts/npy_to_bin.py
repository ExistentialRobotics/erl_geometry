import numpy as np
import argparse
import pathlib


def npy_to_bin(npy_file_path: pathlib.Path, flatten: bool = False):
    data = np.load(npy_file_path, allow_pickle=True)
    folder = npy_file_path.parent
    filename = npy_file_path.stem

    if flatten:
        print(f"Flatten the array of shape {data.shape} to 1D array.")
        data = data.flatten()

    if data.ndim == 1:
        data = data.reshape(1, -1)

    print(f"Data type: {data.dtype}, shape: {data.shape}")

    dat_file = folder / f"{filename}.dat"
    print(f"Converting {npy_file_path} to {dat_file} ... ", end="")
    with open(dat_file, "wb") as f:
        f.write(np.array(data.size).astype(np.int64).tobytes())
        f.write(np.array(data.shape[::-1]).astype(np.int64).tobytes())  # for Eigen default column major
        f.write(data.tobytes("C"))
    print(f"Done.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--npy-file", type=pathlib.Path, required=True, help="Path to the .npy file")
    parser.add_argument("--flatten", action="store_true", help="Flatten the array before saving")
    args = parser.parse_args()
    npy_file_path = args.npy_file
    if not npy_file_path.exists():
        raise FileNotFoundError(f"File {npy_file_path} does not exist.")
    npy_to_bin(npy_file_path, flatten=args.flatten)
    print(f"Converted {npy_file_path} to binary files.")


if __name__ == "__main__":
    main()
