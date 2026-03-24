#!/usr/bin/env python3
"""
Prepare custom RGB / RGB-D / Mono-Inertial inputs for ORB-SLAM3.

Given a dataset with:
- RGB images named by frame_index (e.g. 00001.jpg)
- Depth images named by frame_index (e.g. 00001.png)
- imu.csv mapping frame_index -> reference timestamp
- imu_raw.csv containing raw accel / gyro streams

This script generates a self-contained ORB-SLAM3-ready folder under
custom_dataset_tools/zyc_runner/output/<sequence_name>/.
"""

from __future__ import annotations

import argparse
import csv
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple


IMAGE_EXTS = {".png", ".jpg", ".jpeg", ".bmp", ".tiff", ".webp"}


@dataclass
class ImuPair:
    ts_us: int
    accel: Optional[Tuple[float, float, float]] = None
    gyro: Optional[Tuple[float, float, float]] = None


def sorted_image_files(folder: Path) -> List[Path]:
    files = [p for p in folder.iterdir() if p.is_file() and p.suffix.lower() in IMAGE_EXTS]

    def key_fn(p: Path) -> Tuple[int, str]:
        stem = p.stem
        if stem.isdigit():
            return (0, f"{int(stem):020d}")
        return (1, stem)

    return sorted(files, key=key_fn)


def frame_to_timestamp_ns(imu_csv: Path) -> Dict[str, int]:
    mapping: Dict[str, int] = {}
    with imu_csv.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        required = {"frame_index", "ref_ts_us"}
        if not required.issubset(set(reader.fieldnames or [])):
            raise ValueError(
                f"{imu_csv} missing required columns {sorted(required)}; got {reader.fieldnames}"
            )

        for row in reader:
            frame = str(row["frame_index"]).strip()
            ref_ts_us = int(str(row["ref_ts_us"]).strip())
            mapping[frame] = ref_ts_us * 1000

    return mapping


def load_imu_raw_pairs(imu_raw_csv: Path) -> List[ImuPair]:
    pairs: Dict[int, ImuPair] = {}
    with imu_raw_csv.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        required = {"ts_us", "sensor", "x", "y", "z"}
        if not required.issubset(set(reader.fieldnames or [])):
            raise ValueError(
                f"{imu_raw_csv} missing required columns {sorted(required)}; got {reader.fieldnames}"
            )

        for row in reader:
            ts_us = int(str(row["ts_us"]).strip())
            sensor = str(row["sensor"]).strip().lower()
            xyz = (
                float(str(row["x"]).strip()),
                float(str(row["y"]).strip()),
                float(str(row["z"]).strip()),
            )
            if ts_us not in pairs:
                pairs[ts_us] = ImuPair(ts_us=ts_us)

            if sensor == "accel":
                pairs[ts_us].accel = xyz
            elif sensor == "gyro":
                pairs[ts_us].gyro = xyz

    completed = [p for _, p in sorted(pairs.items(), key=lambda kv: kv[0]) if p.accel and p.gyro]
    return completed


def load_imu_raw_pairs_with_filters(
    imu_raw_csv: Path, time_offset_us: int = 0, skip_before_us: int = 0
) -> List[ImuPair]:
    """Load IMU raw pairs with optional time offset and skip-before filtering.
    
    Args:
        imu_raw_csv: Path to imu_raw.csv
        time_offset_us: Global offset to add to all IMU timestamps (in microseconds)
        skip_before_us: Skip all IMU samples with ts_us < skip_before_us (in microseconds)
    """
    pairs: Dict[int, ImuPair] = {}
    with imu_raw_csv.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        required = {"ts_us", "sensor", "x", "y", "z"}
        if not required.issubset(set(reader.fieldnames or [])):
            raise ValueError(
                f"{imu_raw_csv} missing required columns {sorted(required)}; got {reader.fieldnames}"
            )

        for row in reader:
            ts_us = int(str(row["ts_us"]).strip())
            
            # Apply time offset and skip
            if skip_before_us > 0 and ts_us < skip_before_us:
                continue
            ts_us += time_offset_us
            
            sensor = str(row["sensor"]).strip().lower()
            xyz = (
                float(str(row["x"]).strip()),
                float(str(row["y"]).strip()),
                float(str(row["z"]).strip()),
            )
            if ts_us not in pairs:
                pairs[ts_us] = ImuPair(ts_us=ts_us)

            if sensor == "accel":
                pairs[ts_us].accel = xyz
            elif sensor == "gyro":
                pairs[ts_us].gyro = xyz

    completed = [p for _, p in sorted(pairs.items(), key=lambda kv: kv[0]) if p.accel and p.gyro]
    return completed


def link_or_copy(src: Path, dst: Path, prefer_symlink: bool) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    if dst.exists() or dst.is_symlink():
        dst.unlink()

    if prefer_symlink:
        try:
            dst.symlink_to(src.resolve())
            return
        except OSError:
            pass

    # Fallback to byte copy. For VI images we may use .png extension even if
    # source is jpg; OpenCV decodes by file contents, so this still works.
    shutil.copy2(src, dst)


def write_mono_files(rgb_pairs: List[Tuple[int, Path]], out_dir: Path, prefer_symlink: bool) -> None:
    rgb_dir = out_dir / "rgb"
    rgb_dir.mkdir(parents=True, exist_ok=True)

    rgb_txt = out_dir / "rgb.txt"
    with rgb_txt.open("w", encoding="utf-8", newline="\n") as f:
        f.write("# color images\n")
        f.write("# timestamp filename\n")
        f.write("# generated by prepare_zyc_dataset.py\n")
        for ts_ns, src in rgb_pairs:
            rel = Path("rgb") / src.name
            link_or_copy(src, out_dir / rel, prefer_symlink)
            f.write(f"{ts_ns / 1e9:.9f} {rel.as_posix()}\n")


def write_rgbd_files(
    rgbd_pairs: List[Tuple[int, Path, Path]], out_dir: Path, prefer_symlink: bool
) -> None:
    rgb_dir = out_dir / "rgb"
    depth_dir = out_dir / "depth"
    rgb_dir.mkdir(parents=True, exist_ok=True)
    depth_dir.mkdir(parents=True, exist_ok=True)

    assoc = out_dir / "associations.txt"
    with assoc.open("w", encoding="utf-8", newline="\n") as f:
        for ts_ns, rgb_src, depth_src in rgbd_pairs:
            rgb_rel = Path("rgb") / rgb_src.name
            depth_rel = Path("depth") / depth_src.name
            link_or_copy(rgb_src, out_dir / rgb_rel, prefer_symlink)
            link_or_copy(depth_src, out_dir / depth_rel, prefer_symlink)
            ts_sec = ts_ns / 1e9
            f.write(
                f"{ts_sec:.9f} {rgb_rel.as_posix()} {ts_sec:.9f} {depth_rel.as_posix()}\n"
            )


def write_vi_files(
    rgb_pairs: List[Tuple[int, Path]], imu_pairs: List[ImuPair], out_dir: Path, prefer_symlink: bool
) -> None:
    image_dir = out_dir / "image"
    image_dir.mkdir(parents=True, exist_ok=True)

    times_txt = out_dir / "times_ns.txt"
    with times_txt.open("w", encoding="utf-8", newline="\n") as f:
        for ts_ns, src in rgb_pairs:
            # mono_inertial_tum_vi expects image_path/<timestamp>.png
            vi_name = f"{ts_ns}.png"
            link_or_copy(src, image_dir / vi_name, prefer_symlink)
            f.write(f"{ts_ns}\n")

    imu_csv = out_dir / "imu_tumvi.csv"
    with imu_csv.open("w", encoding="utf-8", newline="\n") as f:
        for p in imu_pairs:
            gx, gy, gz = p.gyro or (0.0, 0.0, 0.0)
            ax, ay, az = p.accel or (0.0, 0.0, 0.0)
            f.write(f"{p.ts_us * 1000},{gx:.9f},{gy:.9f},{gz:.9f},{ax:.9f},{ay:.9f},{az:.9f}\n")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate ORB-SLAM3 inputs for RGB, RGB-D and Mono-Inertial modes"
    )
    parser.add_argument("--data-root", required=True, help="Input sequence folder (contains RGB/Depth/IMU)")
    parser.add_argument("--sequence-name", default="seq", help="Output sequence name under output/")
    parser.add_argument("--rgb-subdir", default="RGB", help="RGB folder name under data-root")
    parser.add_argument("--depth-subdir", default="Depth", help="Depth folder name under data-root")
    parser.add_argument("--imu-subdir", default="IMU", help="IMU folder name under data-root")
    parser.add_argument("--imu-file", default="imu.csv", help="Frame-timestamp IMU csv under imu-subdir")
    parser.add_argument("--imu-raw-file", default="imu_raw.csv", help="Raw IMU csv under imu-subdir")
    parser.add_argument(
        "--copy",
        action="store_true",
        help="Copy files instead of symlink (default prefers symlink and falls back to copy)",
    )
    parser.add_argument(
        "--imu-time-offset-us",
        type=int,
        default=0,
        help="Global IMU timestamp offset in microseconds (use if imu_raw.csv time is ahead/behind frame timestamps)",
    )
    parser.add_argument(
        "--skip-imu-before-us",
        type=int,
        default=0,
        help="Skip IMU samples before this timestamp (microseconds), useful if device was moving at start",
    )

    args = parser.parse_args()

    script_dir = Path(__file__).resolve().parent
    output_root = script_dir / "output" / args.sequence_name

    data_root = Path(args.data_root).resolve()
    rgb_dir = data_root / args.rgb_subdir
    depth_dir = data_root / args.depth_subdir
    imu_dir = data_root / args.imu_subdir
    imu_csv = imu_dir / args.imu_file
    imu_raw_csv = imu_dir / args.imu_raw_file

    for p in [rgb_dir, depth_dir, imu_csv, imu_raw_csv]:
        if not p.exists():
            raise FileNotFoundError(f"Required path does not exist: {p}")

    frame_ts_ns = frame_to_timestamp_ns(imu_csv)

    rgb_files = sorted_image_files(rgb_dir)
    depth_files = sorted_image_files(depth_dir)
    depth_map = {p.stem: p for p in depth_files}

    rgb_pairs: List[Tuple[int, Path]] = []
    rgbd_pairs: List[Tuple[int, Path, Path]] = []
    skipped_no_ts = 0
    skipped_no_depth = 0

    for rgb in rgb_files:
        frame = rgb.stem
        ts_ns = frame_ts_ns.get(frame)
        if ts_ns is None:
            skipped_no_ts += 1
            continue

        rgb_pairs.append((ts_ns, rgb))

        d = depth_map.get(frame)
        if d is not None:
            rgbd_pairs.append((ts_ns, rgb, d))
        else:
            skipped_no_depth += 1

    rgb_pairs.sort(key=lambda x: x[0])
    rgbd_pairs.sort(key=lambda x: x[0])

    imu_pairs = load_imu_raw_pairs_with_filters(
        imu_raw_csv,
        time_offset_us=args.imu_time_offset_us,
        skip_before_us=args.skip_imu_before_us,
    )

    if not rgb_pairs:
        raise RuntimeError("No RGB frames matched imu.csv frame_index -> ref_ts_us")
    if not rgbd_pairs:
        raise RuntimeError("No RGB-Depth frame pairs found with matching frame_index")
    if not imu_pairs:
        raise RuntimeError("No complete accel+gyro samples found in imu_raw.csv")

    # Ensure deterministic output by recreating sequence directory.
    if output_root.exists():
        shutil.rmtree(output_root)

    write_mono_files(rgb_pairs, output_root / "mono", prefer_symlink=not args.copy)
    write_rgbd_files(rgbd_pairs, output_root / "rgbd", prefer_symlink=not args.copy)
    write_vi_files(rgb_pairs, imu_pairs, output_root / "vi", prefer_symlink=not args.copy)

    report = output_root / "prepare_report.txt"
    with report.open("w", encoding="utf-8", newline="\n") as f:
        f.write(f"data_root: {data_root}\n")
        f.write(f"sequence_name: {args.sequence_name}\n")
        f.write(f"imu_time_offset_us: {args.imu_time_offset_us}\n")
        f.write(f"skip_imu_before_us: {args.skip_imu_before_us}\n")
        f.write(f"rgb_frames_used: {len(rgb_pairs)}\n")
        f.write(f"rgbd_pairs_used: {len(rgbd_pairs)}\n")
        f.write(f"imu_samples_used: {len(imu_pairs)}\n")
        f.write(f"skipped_rgb_without_timestamp: {skipped_no_ts}\n")
        f.write(f"skipped_rgb_without_depth: {skipped_no_depth}\n")

    print(f"[OK] Prepared sequence at: {output_root}")
    print(f"[OK] Report: {report}")


if __name__ == "__main__":
    main()
