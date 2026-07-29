#!/usr/bin/env python3
# 中文说明：验证仿真 C10 原始图像数据集的数量、尺寸和目录契约。
# 校验失败只报告数据问题，不修改图像、不启动模型推理。
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
