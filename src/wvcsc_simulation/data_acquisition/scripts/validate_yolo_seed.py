#!/usr/bin/env python3
"""Validate the delivered 30-image unlabelled C10 dataset."""

import argparse

from wvcsc_simulation.data_acquisition.yolo_seed_dataset import validate_unlabeled_dataset


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('dataset')
    args = parser.parse_args()
    totals = validate_unlabeled_dataset(args.dataset)
    print('valid dataset:', totals)


if __name__ == '__main__':
    main()
