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

**可选参数**：

- `--copy`：强制复制文件而不使用软链接
- `--imu-time-offset-us <偏移>` ：若 IMU 时间戳与图像有固定偏移，加上此参数（单位微秒）
- `--skip-imu-before-us <时间戳>`：跳过该时间戳前的 IMU 数据，用于排除录制开始时的动态阶段

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

## 10. IMU 初始化失败排查

如果运行 IMU 模式时看到反复的"IMU is not or recently initialized. Reseting active map"，通常是以下原因之一：

### 第一步：诊断数据对齐

```bash
cd /home/ubuntu/WorkSpace/ZYC/cam_pose/ORB-SLAM3/custom_dataset_tools/zyc_runner
python3 diagnose_vi_data.py --vi-dir output/test1_1_07/vi
```

检查输出中的 `[ERROR]` 和 `[WARN]` 标记，特别关注：

- **时间对齐**：IMU 必须在第一帧图像之前有至少 200+ 样本（1秒 @ 200Hz），否则初始化无法进行
- **时间戳连续性**：查看是否有大的时间跳跃（> 10ms）
- **数据质量**：检查 IMU 原始值是否合理（Z 轴加速度应接近 ±9.81 m/s²，初始时陀螺仪应接近 0）

### 第二步：常见修复

**问题 1：IMU 开始时间晚于图像**

原因：imu.csv 和 imu_raw.csv 的时间戳不同步或有偏移。  
修复：使用诊断结果中的时间偏差，重新预处理数据：

```bash
# 假设诊断显示 IMU 晚图像 500ms，则加上负偏移
python3 prepare_zyc_dataset.py \
  --data-root /home/ubuntu/WorkSpace/ZYC/dataset/imu_joint_calibration/test1/1/07 \
  --sequence-name test1_1_07_fixed \
  --imu-time-offset-us -500000

# 或若 IMU 早图像 200ms，则加上正偏移
python3 prepare_zyc_dataset.py \
  --data-root /home/ubuntu/WorkSpace/ZYC/dataset/imu_joint_calibration/test1/1/07 \
  --sequence-name test1_1_07_fixed \
  --imu-time-offset-us 200000
```

然后用新的 `test1_1_07_fixed` 重新运行 IMU 建图。

**问题 2：IMU 样本不足**

原因：采样时间窗口太短或采样率低。  
修复：确保整个 imu_raw.csv 覆盖从最早 IMU 时间戳到最后一帧的时间范围。

**问题 3：初始时有太多运动**

原因：录制开始时相机/设备在运动，IMU 初始化检测失败。  
修复：用 `--skip-imu-before-us` 跳过前几秒的动态数据：

```bash
# 跳过前 2 秒（2000000 微秒）的 IMU 数据
python3 prepare_zyc_dataset.py \
  --data-root /home/ubuntu/WorkSpace/ZYC/dataset/imu_joint_calibration/test1/1/07 \
  --sequence-name test1_1_07_static \
  --skip-imu-before-us 2000000

# 或同时应用时间偏移和跳过
python3 prepare_zyc_dataset.py \
  --data-root /home/ubuntu/WorkSpace/ZYC/dataset/imu_joint_calibration/test1/1/07 \
  --sequence-name test1_1_07_tuned \
  --imu-time-offset-us -500000 \
  --skip-imu-before-us 1000000
```

**问题 4：YAML 参数不匹配**

在 cam07.yaml 中检查：
- `Imu.NoiseGyro`、`Imu.NoiseAcc`：应与实际 IMU 传感器的规格相符
- `Imu.NoiseGyroWalk`、`Imu.NoiseAccWalk`：偏置漂移参数
- `Camera.Tcb`：相机到 IMU 的外参变换，应对应你的硬件实际安装

### 第三步：跳过 IMU 初始化强制运行（调试用）

若要排除 IMU 初始化问题，临时改用 RGB 不含 IMU 的模式：

```bash
bash custom_dataset_tools/zyc_runner/run_rgb.sh \
  /home/ubuntu/WorkSpace/ZYC/cam_pose/ORB-SLAM3 \
  /home/ubuntu/WorkSpace/ZYC/cam_pose/ORB-SLAM3/Examples/Monocular/cam07.yaml \
  /home/ubuntu/WorkSpace/ZYC/cam_pose/ORB-SLAM3/custom_dataset_tools/zyc_runner/output/test1_1_07
```

若 RGB 模式成功，说明视觉前端和标定没问题，重点在 IMU 数据/参数。
