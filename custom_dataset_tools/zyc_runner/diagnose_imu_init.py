#!/usr/bin/env python3
"""
Detailed IMU initialization diagnostics for ORB-SLAM3.
Checks gravity alignment and bias estimation feasibility.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import List, Tuple


def load_imu_csv(imu_file: Path) -> Tuple[List[int], List[Tuple[float, float, float]], List[Tuple[float, float, float]]]:
    """Load IMU data: timestamp_ns,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z"""
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
            except (ValueError, IndexError):
                pass

    return timestamps, gyro_data, accel_data


def analyze_gravity_alignment(accel_data: List[Tuple[float, float, float]]) -> None:
    """Check if gravity is aligned with one axis (Z should be dominant)."""
    print("\n=== GRAVITY ALIGNMENT ANALYSIS ===")
    
    # Take samples from middle of sequence (more stable)
    start_idx = len(accel_data) // 4
    end_idx = 3 * len(accel_data) // 4
    stable_accel = accel_data[start_idx:end_idx]
    
    if not stable_accel:
        print("[ERROR] Not enough stable samples.")
        return
    
    ax_vals = [a[0] for a in stable_accel]
    ay_vals = [a[1] for a in stable_accel]
    az_vals = [a[2] for a in stable_accel]
    
    ax_mean, ay_mean, az_mean = sum(ax_vals) / len(ax_vals), sum(ay_vals) / len(ay_vals), sum(az_vals) / len(az_vals)
    
    # Calculate magnitude
    g_magnitude = math.sqrt(ax_mean**2 + ay_mean**2 + az_mean**2)
    
    print(f"Stable-phase acceleration (mean): X={ax_mean:.3f}, Y={ay_mean:.3f}, Z={az_mean:.3f} m/s^2")
    print(f"Magnitude: {g_magnitude:.3f} m/s^2 (expect ~9.81)")
    
    if abs(g_magnitude - 9.81) > 1.0:
        print(f"[WARN] Magnitude {g_magnitude:.2f} is far from gravity {9.81:.2f}")
        print(f"       May indicate incorrect IMU scale or units.")
    
    # Check which axis dominates
    abs_x, abs_y, abs_z = abs(ax_mean), abs(ay_mean), abs(az_mean)
    max_component = max(abs_x, abs_y, abs_z)
    min_component = min(abs_x, abs_y, abs_z)
    
    alignment_ratio = max_component / (min_component + 0.001)
    
    print(f"\nDominant axis: ", end="")
    if max_component == abs_x:
        print("X")
    elif max_component == abs_y:
        print("Y")
    else:
        print("Z")
    
    print(f"Alignment ratio (max/min): {alignment_ratio:.2f}")
    
    if alignment_ratio > 5:
        print("[GOOD] Gravity alignment is clear (one axis dominates).")
    else:
        print("[WARN] Gravity is not clearly aligned with any axis.")
        print("[WARN] This suggests incorrect Camera.Tcb (camera-to-IMU rotation).")
        print("[HINT] Check if IMU is rotated relative to camera expectation.")
    
    # Check sign
    if az_mean < -5:
        print(f"[OK] Z-axis is negative ({az_mean:.2f}), typical for forward-facing camera.")
    elif az_mean > 5:
        print(f"[WARN] Z-axis is strongly positive ({az_mean:.2f}), camera may be inverted?")
    else:
        print(f"[ERROR] Z-axis is close to zero ({az_mean:.2f}), gravity NOT on Z-axis!")
        print("[HINT] IMU coordinate frame appears completely misaligned.")


def analyze_gyro_bias(gyro_data: List[Tuple[float, float, float]]) -> None:
    """Check if gyroscope bias is stable and estimable."""
    print("\n=== GYROSCOPE BIAS ANALYSIS ===")
    
    # Use stable middle section
    start_idx = len(gyro_data) // 4
    end_idx = 3 * len(gyro_data) // 4
    stable_gyro = gyro_data[start_idx:end_idx]
    
    if not stable_gyro:
        print("[ERROR] Not enough gyro samples.")
        return
    
    gx_vals = [g[0] for g in stable_gyro]
    gy_vals = [g[1] for g in stable_gyro]
    gz_vals = [g[2] for g in stable_gyro]
    
    gx_mean, gy_mean, gz_mean = sum(gx_vals) / len(gx_vals), sum(gy_vals) / len(gy_vals), sum(gz_vals) / len(gz_vals)
    gx_std = (sum((x - gx_mean)**2 for x in gx_vals) / len(gx_vals))**0.5
    gy_std = (sum((y - gy_mean)**2 for y in gy_vals) / len(gy_vals))**0.5
    gz_std = (sum((z - gz_mean)**2 for z in gz_vals) / len(gz_vals))**0.5
    
    print(f"Gyro bias estimates (stable phase):")
    print(f"  X: {gx_mean:.6f} ± {gx_std:.6f} rad/s")
    print(f"  Y: {gy_mean:.6f} ± {gy_std:.6f} rad/s")
    print(f"  Z: {gz_mean:.6f} ± {gz_std:.6f} rad/s")
    
    # Check if bias is reasonable
    bias_mag = math.sqrt(gx_mean**2 + gy_mean**2 + gz_mean**2)
    if bias_mag < 0.1:
        print(f"[GOOD] Gyro bias is small ({bias_mag:.6f} rad/s).")
    else:
        print(f"[WARN] Gyro bias is large ({bias_mag:.6f} rad/s).")
    
    # Check noise level
    noise_level = (gx_std + gy_std + gz_std) / 3
    print(f"Average gyro noise (std): {noise_level:.6f} rad/s")
    if noise_level < 0.05:
        print("[GOOD] Gyro noise is low.")
    elif noise_level < 0.2:
        print("[OK] Gyro noise is moderate.")
    else:
        print("[WARN] Gyro noise is high, may degrade initialization.")


def analyze_accel_variance(accel_data: List[Tuple[float, float, float]]) -> None:
    """Check acceleration variance (expect low variance if device is still)."""
    print("\n=== ACCELERATION VARIANCE ANALYSIS ===")
    
    first_100 = accel_data[:100]
    if len(first_100) < 100:
        print("[WARN] Less than 100 samples available.")
        first_100 = accel_data
    
    ax_std = (sum((a[0] - sum(a[0] for a in first_100)/len(first_100))**2 for a in first_100) / len(first_100))**0.5
    ay_std = (sum((a[1] - sum(a[1] for a in first_100)/len(first_100))**2 for a in first_100) / len(first_100))**0.5
    az_std = (sum((a[2] - sum(a[2] for a in first_100)/len(first_100))**2 for a in first_100) / len(first_100))**0.5
    
    print(f"First 100 samples acceleration variance (std):")
    print(f"  X: {ax_std:.4f} m/s^2")
    print(f"  Y: {ay_std:.4f} m/s^2")
    print(f"  Z: {az_std:.4f} m/s^2")
    
    avg_std = (ax_std + ay_std + az_std) / 3
    if avg_std < 0.1:
        print(f"[GOOD] Acceleration variance is very low ({avg_std:.4f}), device is stationary.")
    elif avg_std < 0.5:
        print(f"[OK] Acceleration variance is low ({avg_std:.4f}).")
    else:
        print(f"[WARN] Acceleration variance is high ({avg_std:.4f}), device may be moving.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Deep IMU initialization diagnostics")
    parser.add_argument("--imu-file", required=True, help="Path to imu_tumvi.csv")
    args = parser.parse_args()

    imu_file = Path(args.imu_file).resolve()
    if not imu_file.exists():
        raise FileNotFoundError(f"IMU file not found: {imu_file}")

    print(f"Loading IMU data from: {imu_file}")
    timestamps, gyro_data, accel_data = load_imu_csv(imu_file)
    
    print(f"Loaded {len(accel_data)} IMU samples, {len(gyro_data)} gyro samples")

    analyze_gravity_alignment(accel_data)
    analyze_gyro_bias(gyro_data)
    analyze_accel_variance(accel_data)

    print("\n=== DIAGNOSIS SUMMARY ===")
    print("If gravity is NOT on Z-axis, your Camera.Tcb rotation matrix is wrong.")
    print("Common issues:")
    print("  1. Camera mounted upside-down → Z-axis points upward")
    print("  2. Camera mounted sideways → gravity on X or Y axis")
    print("  3. IMU coordinate frame different from camera frame")
    print("\nFix:")
    print("  - Verify actual camera-IMU mounting")
    print("  - Update Camera.Tcb rotation to match hardware")
    print("  - Or adjust IMU initialization thresholds in YAML")


if __name__ == "__main__":
    main()
