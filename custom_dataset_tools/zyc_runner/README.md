# ORB-SLAM3 自定义数据运行指南（RGB / RGB-D / IMU）

本目录用于把你自己的数据一键整理为 ORB-SLAM3 可直接运行的输入格式。

## 1. 已创建文件

- `prepare_zyc_dataset.py`：数据预处理脚本
- `run_rgb.sh`：运行 RGB 模式（`mono_tum`）
- `run_rgbd.sh`：运行 RGB-D 模式（`rgbd_tum`）
- `run_imu.sh`：运行视觉-IMU 模式（`mono_inertial_tum_vi`）
- `output/`：预处理输出目录

## 2. 输入数据要求（与你当前一致）

假设序列目录为：

`/home/ubuntu/WorkSpace/ZYC/dataset/imu_joint_calibration/test1/1/07`

其下有：

- `RGB/00001.jpg`（文件名是 frame_index）
- `Depth/00001.png`（文件名是 frame_index）
- `IMU/imu.csv`（包含 `frame_index,ref_ts_us,...`）
- `IMU/imu_raw.csv`（包含 `ts_us,sensor,x,y,z`）

## 3. 先预处理数据

在 Ubuntu 终端执行：

```bash
cd /home/ubuntu/WorkSpace/ZYC/cam_pose/ORB-SLAM3/custom_dataset_tools/zyc_runner
python3 prepare_zyc_dataset.py \
  --data-root /home/ubuntu/WorkSpace/ZYC/dataset/imu_joint_calibration/test1/1/07 \
  --sequence-name test1_1_07
```

完成后会生成：

`/home/ubuntu/WorkSpace/ZYC/cam_pose/ORB-SLAM3/custom_dataset_tools/zyc_runner/output/test1_1_07`

结构如下：

- `mono/`：给 `mono_tum` 用
- `rgbd/`：给 `rgbd_tum` 用
- `vi/`：给 `mono_inertial_tum_vi` 用
- `prepare_report.txt`：统计与跳过帧信息

## 4. 运行 RGB 模式

```bash
cd /home/ubuntu/WorkSpace/ZYC/cam_pose/ORB-SLAM3
bash custom_dataset_tools/zyc_runner/run_rgb.sh \
  /home/ubuntu/WorkSpace/ZYC/cam_pose/ORB-SLAM3 \
  /home/ubuntu/WorkSpace/ZYC/cam_pose/ORB-SLAM3/Examples/Monocular/RealSense_D435i.yaml \
  /home/ubuntu/WorkSpace/ZYC/cam_pose/ORB-SLAM3/custom_dataset_tools/zyc_runner/output/test1_1_07
```

## 5. 运行 RGB-D 模式

```bash
cd /home/ubuntu/WorkSpace/ZYC/cam_pose/ORB-SLAM3
bash custom_dataset_tools/zyc_runner/run_rgbd.sh \
  /home/ubuntu/WorkSpace/ZYC/cam_pose/ORB-SLAM3 \
  /home/ubuntu/WorkSpace/ZYC/cam_pose/ORB-SLAM3/Examples/RGB-D/RealSense_D435i.yaml \
  /home/ubuntu/WorkSpace/ZYC/cam_pose/ORB-SLAM3/custom_dataset_tools/zyc_runner/output/test1_1_07
```

## 6. 运行 IMU（视觉-IMU）模式

```bash
cd /home/ubuntu/WorkSpace/ZYC/cam_pose/ORB-SLAM3
bash custom_dataset_tools/zyc_runner/run_imu.sh \
  /home/ubuntu/WorkSpace/ZYC/cam_pose/ORB-SLAM3 \
  /home/ubuntu/WorkSpace/ZYC/cam_pose/ORB-SLAM3/Examples/Monocular-Inertial/RealSense_D435i.yaml \
  /home/ubuntu/WorkSpace/ZYC/cam_pose/ORB-SLAM3/custom_dataset_tools/zyc_runner/output/test1_1_07 \
  test1_1_07
```

## 7. 重要说明（务必检查）

- 相机内参与畸变、IMU 噪声参数、相机-IMU 外参必须与你设备一致。
- 上面示例里的 `RealSense_D435i.yaml` 只是模板路径，不一定适配你的设备。
- `prepare_zyc_dataset.py` 默认优先使用软链接；若环境不支持软链接会自动回退到复制。
- 若你希望强制复制文件，可在预处理时加 `--copy`。

## 8. IMU 格式转换细节

脚本会把 `imu_raw.csv` 转成 `vi/imu_tumvi.csv`，格式为：

`timestamp_ns,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z`

其中 `timestamp_ns = ts_us * 1000`，并按同一时间戳下 accel/gyro 配对后输出。

## 9. 常见问题

- 报错找不到图片：检查 `output/<seq>/...` 目录是否生成，且 `prepare_report.txt` 的 `rgb_frames_used` 是否大于 0。
- 报错 IMU 初始化失败：重点检查 YAML 的 IMU 噪声和外参。
- 轨迹漂移大：优先检查时间同步、标定参数、深度尺度（RGB-D 模式）。
