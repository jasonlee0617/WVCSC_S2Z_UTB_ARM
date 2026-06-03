#!/usr/bin/env python3
"""Convert a Nav2 occupancy-grid PGM map into a flat fake PCD for temporary Autoware bringup.

This utility is for launch-chain validation only. It should not be used as a final
pointcloud map for NDT localization.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, List, Tuple


def parse_simple_yaml(path: Path) -> Dict[str, str]:
    data: Dict[str, str] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, value = line.split(":", 1)
        data[key.strip()] = value.strip()
    return data


def parse_origin(value: str) -> Tuple[float, float, float]:
    text = value.strip()
    if not (text.startswith("[") and text.endswith("]")):
        raise ValueError(f"Invalid origin field: {value}")
    parts = [item.strip() for item in text[1:-1].split(",")]
    if len(parts) != 3:
        raise ValueError(f"Origin must have 3 values: {value}")
    return float(parts[0]), float(parts[1]), float(parts[2])


def load_pgm(path: Path) -> Tuple[int, int, int, List[int]]:
    with path.open("rb") as f:
        magic = f.readline().strip()
        if magic not in {b"P2", b"P5"}:
            raise ValueError(f"Unsupported PGM format: {magic!r}")

        def next_tokens() -> List[bytes]:
            while True:
                line = f.readline()
                if not line:
                    return []
                line = line.strip()
                if not line or line.startswith(b"#"):
                    continue
                return line.split()

        tokens = next_tokens()
        while len(tokens) < 2:
            tokens.extend(next_tokens())
        width = int(tokens[0])
        height = int(tokens[1])

        max_value_tokens = next_tokens()
        if not max_value_tokens:
            raise ValueError("Missing PGM max value")
        max_value = int(max_value_tokens[0])

        if magic == b"P5":
            payload = f.read()
            expected = width * height
            if len(payload) < expected:
                raise ValueError("PGM payload shorter than expected")
            pixels = list(payload[:expected])
        else:
            remaining = f.read().split()
            pixels = [int(x) for x in remaining]
            expected = width * height
            if len(pixels) < expected:
                raise ValueError("PGM payload shorter than expected")
            pixels = pixels[:expected]

    return width, height, max_value, pixels


def occupancy_probability(pixel: int, max_value: int, negate: int) -> float:
    normalized = pixel / float(max_value)
    if negate:
        return normalized
    return 1.0 - normalized


def build_points(
    width: int,
    height: int,
    max_value: int,
    pixels: List[int],
    resolution: float,
    origin: Tuple[float, float, float],
    occupied_thresh: float,
    negate: int,
    sample_step: int,
) -> List[Tuple[float, float, float]]:
    origin_x, origin_y, origin_z = origin
    points: List[Tuple[float, float, float]] = []
    for row in range(0, height, sample_step):
        for col in range(0, width, sample_step):
            pixel = pixels[row * width + col]
            if occupancy_probability(pixel, max_value, negate) < occupied_thresh:
                continue
            x = origin_x + (col + 0.5) * resolution
            y = origin_y + (height - row - 0.5) * resolution
            z = origin_z
            points.append((x, y, z))
    return points


def write_ascii_pcd(path: Path, points: List[Tuple[float, float, float]]) -> None:
    with path.open("w", encoding="utf-8") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {len(points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\n")
        f.write("DATA ascii\n")
        for x, y, z in points:
            f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert Nav2 PGM/YAML into a temporary flat PCD map")
    parser.add_argument("--yaml", required=True, help="Path to map yaml, for example map_new.yaml")
    parser.add_argument("--output", required=True, help="Output fake pointcloud_map.pcd path")
    parser.add_argument("--sample-step", type=int, default=1, help="Use every Nth occupied pixel to reduce map size")
    args = parser.parse_args()

    yaml_path = Path(args.yaml).expanduser().resolve()
    config = parse_simple_yaml(yaml_path)

    image_name = config.get("image")
    if not image_name:
        raise ValueError("Map yaml is missing image field")

    image_path = (yaml_path.parent / image_name).resolve()
    resolution = float(config.get("resolution", "0.05"))
    origin = parse_origin(config.get("origin", "[0.0, 0.0, 0.0]"))
    occupied_thresh = float(config.get("occupied_thresh", "0.65"))
    negate = int(float(config.get("negate", "0")))

    width, height, max_value, pixels = load_pgm(image_path)
    points = build_points(
        width=width,
        height=height,
        max_value=max_value,
        pixels=pixels,
        resolution=resolution,
        origin=origin,
        occupied_thresh=occupied_thresh,
        negate=negate,
        sample_step=max(1, args.sample_step),
    )

    output_path = Path(args.output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    write_ascii_pcd(output_path, points)

    print("Temporary fake PCD generated")
    print(f"  yaml:    {yaml_path}")
    print(f"  image:   {image_path}")
    print(f"  output:  {output_path}")
    print(f"  points:  {len(points)}")
    print("Warning: this map is only for launch-chain validation, not final NDT deployment.")


if __name__ == "__main__":
    main()
