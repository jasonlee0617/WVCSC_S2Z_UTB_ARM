#!/usr/bin/env python3
"""Safely replace the two raw C10 dataset roots from validated staging data."""

import argparse

from wvcsc_simulation.data_acquisition.yolo_seed_dataset import replace_dataset_pair


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--fruit-staging', required=True)
    parser.add_argument('--tree-staging', required=True)
    parser.add_argument('--fruit-destination', required=True)
    parser.add_argument('--tree-destination', required=True)
    args = parser.parse_args()
    print(replace_dataset_pair(
        args.fruit_staging, args.tree_staging,
        args.fruit_destination, args.tree_destination))


if __name__ == '__main__':
    main()
