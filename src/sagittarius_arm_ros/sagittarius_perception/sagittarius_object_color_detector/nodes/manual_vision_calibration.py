#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Fit vision_config.yaml linear mapping from manually collected points.

CSV columns:
    pixel_x,pixel_y,robot_x,robot_y

The project mapping is:
    robot_x = k1 * pixel_y + b1
    robot_y = k2 * pixel_x + b2
"""

import argparse
import csv
import os
import shutil
import time

import yaml


REQUIRED_COLUMNS = ("pixel_x", "pixel_y", "robot_x", "robot_y")


def read_points(csv_path):
    points = []
    with open(csv_path, "r", newline="") as stream:
        reader = csv.DictReader(stream)
        missing = [name for name in REQUIRED_COLUMNS if name not in reader.fieldnames]
        if missing:
            raise ValueError(
                "CSV is missing required columns: {}".format(", ".join(missing))
            )
        for row_index, row in enumerate(reader, start=2):
            if not any((row.get(name) or "").strip() for name in REQUIRED_COLUMNS):
                continue
            try:
                points.append(
                    {
                        "pixel_x": float(row["pixel_x"]),
                        "pixel_y": float(row["pixel_y"]),
                        "robot_x": float(row["robot_x"]),
                        "robot_y": float(row["robot_y"]),
                    }
                )
            except (TypeError, ValueError) as exc:
                raise ValueError("Invalid numeric value on CSV line {}: {}".format(row_index, exc))
    if len(points) < 2:
        raise ValueError("At least 2 calibration points are required")
    return points


def fit_line(xs, ys):
    mean_x = sum(xs) / float(len(xs))
    mean_y = sum(ys) / float(len(ys))
    numerator = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    denominator = sum((x - mean_x) ** 2 for x in xs)
    if abs(denominator) < 1e-12:
        raise ValueError("Cannot fit line because input values have no variance")
    k = numerator / denominator
    b = mean_y - k * mean_x
    return k, b


def mean_abs_error(xs, ys, k, b):
    return sum(abs((k * x + b) - y) for x, y in zip(xs, ys)) / float(len(xs))


def load_yaml(path):
    with open(path, "r") as stream:
        content = yaml.safe_load(stream)
    if "LinearRegression" not in content:
        content["LinearRegression"] = {}
    return content


def save_yaml_with_backup(path, content):
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    backup_path = "{}.bak_{}".format(path, timestamp)
    shutil.copy2(path, backup_path)
    with open(path, "w") as stream:
        yaml.safe_dump(content, stream, default_flow_style=False, sort_keys=False)
    return backup_path


def main():
    parser = argparse.ArgumentParser(
        description="Fit Sagittarius pixel-to-plane linear mapping from manual CSV points."
    )
    parser.add_argument("--csv", required=True, help="Calibration CSV path")
    parser.add_argument("--vision-config", required=True, help="vision_config.yaml path")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print fitted values without writing vision_config.yaml",
    )
    args = parser.parse_args()

    csv_path = os.path.abspath(args.csv)
    vision_config_path = os.path.abspath(args.vision_config)

    points = read_points(csv_path)
    pixel_x = [point["pixel_x"] for point in points]
    pixel_y = [point["pixel_y"] for point in points]
    robot_x = [point["robot_x"] for point in points]
    robot_y = [point["robot_y"] for point in points]

    # Keep compatibility with the original mapping convention.
    k1, b1 = fit_line(pixel_y, robot_x)
    k2, b2 = fit_line(pixel_x, robot_y)
    x_error = mean_abs_error(pixel_y, robot_x, k1, b1)
    y_error = mean_abs_error(pixel_x, robot_y, k2, b2)

    print("Loaded {} calibration points".format(len(points)))
    print("robot_x = k1 * pixel_y + b1")
    print("  k1 = {:.10f}".format(k1))
    print("  b1 = {:.10f}".format(b1))
    print("  mean abs error x = {:.6f} m".format(x_error))
    print("robot_y = k2 * pixel_x + b2")
    print("  k2 = {:.10f}".format(k2))
    print("  b2 = {:.10f}".format(b2))
    print("  mean abs error y = {:.6f} m".format(y_error))

    if args.dry_run:
        print("Dry run only; vision_config.yaml was not modified")
        return

    content = load_yaml(vision_config_path)
    content["LinearRegression"]["k1"] = float(k1)
    content["LinearRegression"]["b1"] = float(b1)
    content["LinearRegression"]["k2"] = float(k2)
    content["LinearRegression"]["b2"] = float(b2)
    backup_path = save_yaml_with_backup(vision_config_path, content)
    print("Updated: {}".format(vision_config_path))
    print("Backup:  {}".format(backup_path))


if __name__ == "__main__":
    main()
