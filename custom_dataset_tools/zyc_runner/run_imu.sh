#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 3 ]; then
  echo "Usage: $0 <ORB_SLAM3_ROOT> <SETTINGS_YAML> <PREPARED_SEQ_DIR> [traj_name]"
  echo "Example:"
  echo "  $0 /home/ubuntu/WorkSpace/cam_pose/ORB-SLAM3 \\" 
  echo "     /home/ubuntu/WorkSpace/cam_pose/ORB-SLAM3/Examples/Monocular-Inertial/RealSense_D435i.yaml \\" 
  echo "     /home/ubuntu/WorkSpace/cam_pose/ORB-SLAM3/custom_dataset_tools/zyc_runner/output/test1_1_07 myrun"
  exit 1
fi

ORB_ROOT="$1"
SETTINGS="$2"
PREPARED="$3"
TRAJ_NAME="${4:-}"

CMD=(
  "${ORB_ROOT}/Examples/Monocular-Inertial/mono_inertial_tum_vi"
  "${ORB_ROOT}/Vocabulary/ORBvoc.txt"
  "${SETTINGS}"
  "${PREPARED}/vi/image"
  "${PREPARED}/vi/times_ns.txt"
  "${PREPARED}/vi/imu_tumvi.csv"
)

if [ -n "${TRAJ_NAME}" ]; then
  CMD+=("${TRAJ_NAME}")
fi

"${CMD[@]}"
