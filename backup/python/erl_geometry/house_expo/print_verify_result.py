import argparse
import pickle


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--verify-result-pkl", required=True, type=str)
    parser.add_argument("--exclude-results", nargs="+", type=str)
    parser.add_argument("--json-name-only", action="store_true")

    args = parser.parse_args()

    with open(args.verify_result_pkl, "rb") as file:
        results: dict = pickle.load(file)

    for json_name, result in results.items():
        if args.exclude_results is not None:
            excluded = False
            for verf_result in result["verf_result"]:
                if verf_result in args.exclude_results:
                    excluded = True
                    break
            if excluded:
                continue

        if args.json_name_only:
            print(json_name)
            continue

        print(json_name, result)


if __name__ == "__main__":
    main()
