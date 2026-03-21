#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 3 ] || [ "$#" -gt 5 ]; then
  echo "Usage: $0 path_to_vocabulary path_to_settings path_to_sequence_folder [trajectory_file_name] [fps_if_no_imu_csv]"
  echo "Example:"
  echo "  $0 ./Vocabulary/ORBvoc.txt ./Examples/RGB-D/RealSense_D435i.yaml /home/ubuntu/WorkSpace/ZYC/dataset/imu_joint_calibration/test1/1/07 traj_rgbd.txt 30"
  exit 1
fi

VOCAB_PATH="$1"
SETTINGS_PATH="$2"
SEQ_PATH="$3"
TRAJ_NAME="${4:-CameraTrajectory.txt}"
FPS_NO_IMU="${5:-30}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BIN_PATH="$REPO_ROOT/Examples/RGB-D/rgbd_local_dataset"

if [ ! -x "$BIN_PATH" ]; then
  echo "Binary not found: $BIN_PATH"
  echo "Build first:"
  echo "  cd $REPO_ROOT"
  echo "  mkdir -p build && cd build"
  echo "  cmake .."
  echo "  make -j$(nproc)"
  exit 1
fi

"$BIN_PATH" "$VOCAB_PATH" "$SETTINGS_PATH" "$SEQ_PATH" "$TRAJ_NAME" "$FPS_NO_IMU"
