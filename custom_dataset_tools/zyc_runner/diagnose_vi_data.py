#!/usr/bin/env python3
"""
Diagnose VI (Visual-Inertial) data alignment and quality issues.
Checks time synchronization, data continuity, and IMU statistics.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import List, Tuple


def load_times_ns(times_file: Path) -> List[int]:
    """Load nanosecond timestamps from times_ns.txt"""
    times = []
    with times_file.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                times.append(int(line))
    return times


def load_imu_csv(imu_file: Path) -> Tuple[List[int], List[Tuple[float, float, float]], List[Tuple[float, float, float]]]:
    """Load IMU data from imu_tumvi.csv.
    Format: timestamp_ns,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z
    """
    timestamps = []
    gyro_data = []
    accel_data = []

    with imu_file.open("r", newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].startswith("#"):
                continue
            try:
                ts = int(row[0])
                gx, gy, gz = float(row[1]), float(row[2]), float(row[3])
                ax, ay, az = float(row[4]), float(row[5]), float(row[6])
                timestamps.append(ts)
                gyro_data.append((gx, gy, gz))
                accel_data.append((ax, ay, az))
            except (ValueError, IndexError) as e:
                print(f"[WARN] Skipping malformed IMU line: {row} ({e})")

    return timestamps, gyro_data, accel_data


def analyze_time_alignment(image_times: List[int], imu_times: List[int]) -> None:
    """Analyze temporal alignment between images and IMU."""
    print("\n=== TIME ALIGNMENT ANALYSIS ===")
    print(f"Number of images: {len(image_times)}")
    print(f"Number of IMU samples: {len(imu_times)}")

    if not image_times or not imu_times:
        print("[ERROR] Empty image or IMU data.")
        return

    img_start_s = image_times[0] / 1e9
    img_end_s = image_times[-1] / 1e9
    imu_start_s = imu_times[0] / 1e9
    imu_end_s = imu_times[-1] / 1e9

    print(f"\nImage time range: {img_start_s:.3f} - {img_end_s:.3f} s")
    print(f"IMU time range:   {imu_start_s:.3f} - {imu_end_s:.3f} s")

    # Check if IMU starts before first image
    if imu_start_s < img_start_s:
        delta = (img_start_s - imu_start_s) * 1000
        print(f"[GOOD] IMU starts {delta:.1f} ms before first image.")
    else:
        delta = (imu_start_s - img_start_s) * 1000
        print(f"[ERROR] IMU starts {delta:.1f} ms AFTER first image!")
        print(f"        [FIX] IMU initialization needs data before first frame.")

    # Check coverage
    if imu_end_s >= img_end_s:
        print(f"[GOOD] IMU covers entire image sequence.")
    else:
        missing = (img_end_s - imu_end_s) * 1000
        print(f"[WARN] IMU ends {missing:.1f} ms before last image.")

    # Expected IMU count
    img_duration = img_end_s - img_start_s
    expected_imu_count = int(img_duration * 200)  # Assuming 200 Hz IMU
    actual_imu_count = len(imu_times)
    print(f"\nExpected IMU samples (@ 200 Hz): ~{expected_imu_count}")
    print(f"Actual IMU samples:              {actual_imu_count}")
    if actual_imu_count > expected_imu_count * 0.8:
        print(f"[GOOD] IMU sample count is reasonable.")
    else:
        print(f"[WARN] IMU sample count is low (only {100*actual_imu_count/expected_imu_count:.1f}% of expected).")


def analyze_imu_gaps(imu_times: List[int]) -> None:
    """Check for gaps or jumps in IMU timestamps."""
    print("\n=== IMU TIMESTAMP CONTINUITY ===")
    
    if len(imu_times) < 2:
        print("[WARN] Not enough IMU samples to check continuity.")
        return

    gaps = []
    for i in range(1, len(imu_times)):
        dt_ns = imu_times[i] - imu_times[i - 1]
        dt_ms = dt_ns / 1e6
        if dt_ms > 10:  # More than 10 ms = abnormal at 200 Hz
            gaps.append((i - 1, i, dt_ms))

    if gaps:
        print(f"[WARN] Found {len(gaps)} timestamp jumps > 10 ms:")
        for idx_before, idx_after, dt_ms in gaps[:5]:  # Show first 5
            print(f"       Between sample {idx_before}-{idx_after}: {dt_ms:.2f} ms")
        if len(gaps) > 5:
            print(f"       ... and {len(gaps) - 5} more")
    else:
        print("[GOOD] IMU timestamps are continuous (no jumps > 10 ms).")


def analyze_imu_statistics(gyro_data: List[Tuple[float, float, float]], 
                           accel_data: List[Tuple[float, float, float]]) -> None:
    """Check IMU data statistics for anomalies."""
    print("\n=== IMU DATA STATISTICS ===")

    gyro_x = [g[0] for g in gyro_data]
    gyro_y = [g[1] for g in gyro_data]
    gyro_z = [g[2] for g in gyro_data]
    accel_x = [a[0] for a in accel_data]
    accel_y = [a[1] for a in accel_data]
    accel_z = [a[2] for a in accel_data]

    def print_stats(name: str, values: List[float]) -> None:
        if not values:
            print(f"{name}: [EMPTY]")
            return
        min_v, max_v, mean_v = min(values), max(values), sum(values) / len(values)
        print(f"{name}:")
        print(f"  Range: [{min_v:.6f}, {max_v:.6f}]")
        print(f"  Mean:  {mean_v:.6f}")

    print("Gyroscope (rad/s):")
    print_stats("  X", gyro_x)
    print_stats("  Y", gyro_y)
    print_stats("  Z", gyro_z)

    print("Accelerometer (m/s^2):")
    print_stats("  X", accel_x)
    print_stats("  Y", accel_y)
    print_stats("  Z", accel_z)

    # Check for static initialization
    g_magnitude = [((gx**2 + gy**2 + gz**2)**0.5) for gx, gy, gz in gyro_data[:100]]
    g_mean = sum(g_magnitude) / len(g_magnitude) if g_magnitude else 0
    if g_mean < 0.01:
        print(f"\n[GOOD] First 100 gyro samples show low motion (mean magnitude: {g_mean:.6f} rad/s).")
    else:
        print(f"\n[WARN] First 100 gyro samples show high motion (mean magnitude: {g_mean:.6f} rad/s).")
        print(f"       IMU initialization may fail if sensor is moving at startup.")

    a_z_list = accel_z[:100]
    a_z_mean = sum(a_z_list) / len(a_z_list) if a_z_list else 0
    if abs(a_z_mean - (-9.81)) < 1.0:
        print(f"[GOOD] Accelerometer Z is near gravity ({a_z_mean:.2f} m/s^2 ≈ -9.81).")
    elif abs(a_z_mean - 9.81) < 1.0:
        print(f"[WARN] Accelerometer Z is {a_z_mean:.2f} m/s^2 (inverted gravity?).")
    else:
        print(f"[WARN] Accelerometer Z is {a_z_mean:.2f} m/s^2 (expect ±9.81).")


def main() -> None:
    parser = argparse.ArgumentParser(description="Diagnose VI data alignment and quality")
    parser.add_argument("--vi-dir", required=True, help="Path to vi/ output directory")
    args = parser.parse_args()

    vi_dir = Path(args.vi_dir).resolve()
    times_file = vi_dir / "times_ns.txt"
    imu_file = vi_dir / "imu_tumvi.csv"

    for f in [times_file, imu_file]:
        if not f.exists():
            raise FileNotFoundError(f"Required file not found: {f}")

    print(f"Loading data from: {vi_dir}")
    image_times = load_times_ns(times_file)
    imu_times, gyro_data, accel_data = load_imu_csv(imu_file)

    analyze_time_alignment(image_times, imu_times)
    analyze_imu_gaps(imu_times)
    analyze_imu_statistics(gyro_data, accel_data)

    print("\n=== SUMMARY ===")
    print("If you see [ERROR] or [WARN], the issues may cause IMU initialization failure.")
    print("Common fixes:")
    print("  1. Ensure IMU data starts BEFORE first image (with 200+ samples margin)")
    print("  2. Check for time jumps or gaps in IMU timestamps")
    print("  3. Verify the device was stationary at the start of recording")
    print("  4. Adjust IMU noise parameters in YAML if data quality is poor")


if __name__ == "__main__":
    main()
