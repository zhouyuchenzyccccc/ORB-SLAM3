#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 3 ]; then
  echo "Usage: $0 <ORB_SLAM3_ROOT> <SETTINGS_YAML> <PREPARED_SEQ_DIR>"
  echo "Example:"
  echo "  $0 /home/ubuntu/WorkSpace/cam_pose/ORB-SLAM3 \\" 
  echo "     /home/ubuntu/WorkSpace/cam_pose/ORB-SLAM3/Examples/Monocular/RealSense_D435i.yaml \\" 
  echo "     /home/ubuntu/WorkSpace/cam_pose/ORB-SLAM3/custom_dataset_tools/zyc_runner/output/test1_1_07"
  exit 1
fi

ORB_ROOT="$1"
SETTINGS="$2"
PREPARED="$3"

"${ORB_ROOT}/Examples/Monocular/mono_tum" \
  "${ORB_ROOT}/Vocabulary/ORBvoc.txt" \
  "${SETTINGS}" \
  "${PREPARED}/mono"
