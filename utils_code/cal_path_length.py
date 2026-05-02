#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import os

def calculate_trajectory_length(file_path):
    if not os.path.exists(file_path):
        print(f"Error: file {file_path} does not exist")
        return

    try:
        data = np.loadtxt(file_path)
    except Exception as e:
        print(f"Failed to parse file: {e}")
        return

    if data.shape[0] < 2:
        print("Not enough trajectory points to compute length.")
        return 0.0

    positions = data[:, 1:4]
    diffs = np.diff(positions, axis=0)
    distances = np.linalg.norm(diffs, axis=1)
    total_length = np.sum(distances)

    print("-" * 30)
    print(f"File: {os.path.basename(file_path)}")
    print(f"Total points: {len(positions)}")
    print(f"Total duration: {data[-1, 0] - data[0, 0]:.2f} s")
    print(f"Total trajectory length: {total_length:.4f} m")
    print("-" * 30)

    return total_length

if __name__ == "__main__":
    file_to_read = "GT-Traj-PoolM#10.txt"
    calculate_trajectory_length(file_to_read)