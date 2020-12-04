import argparse
import pstats


def main(args):
    p = pstats.Stats(args.profile_path)
    p.strip_dirs().sort_stats("tottime").print_stats(20)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile_path", type=str, default="./profile.stats")
    args = parser.parse_args()

    main(args)
