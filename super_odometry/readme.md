## Super Odometry

Super Odometry is a high-performance odometry library designed for robotics applications. It integrates advanced algorithms for real-time localization and mapping, supporting a wide range of sensors.

## TartanAir Quick Run

Use `config/tartan_air.yaml` as the current TartanAir baseline config.

LiDAR+IMU only:
```bash
ros2 launch super_odometry tartan_air.launch.py \
  enable_visual_odometry:=false \
  enable_visual_fusion:=false \
  enable_visual_deskew:=false
```

VO running but not fused:
```bash
ros2 launch super_odometry tartan_air.launch.py \
  enable_visual_odometry:=true \
  enable_visual_fusion:=false \
  enable_visual_deskew:=false
```

Conservative fusion:
```bash
ros2 launch super_odometry tartan_air.launch.py \
  enable_visual_odometry:=true \
  enable_visual_fusion:=true \
  enable_visual_deskew:=false \
  use_vio_when_not_degenerate:=false \
  sync_tolerance_sec:=0.2 \
  odom_interp_slack_sec:=0.2
```

Interpretation of the conservative fusion run:
- `visual_odometry_node` is running.
- Visual fusion is enabled.
- `laser_odom_path` is still mostly the LIO-driven mapped output.
- VIO is only allowed to contribute when visual odometry is valid and mapping selects `PredictionSource::VIO_ODOM`, mainly as a conservative fallback in degenerate situations.
- IMU deskew remains the preferred LiDAR deskew path in this setup.

## How to Run the localization mode? 
Please change the yaml file 
```
        localization_mode: true             # if true, localization mode is on; Otherwise, SLAM mode is on  
        read_pose_file: false        # read the txt pose as the initial pose for localization
        init_x: 13.983960            # initial pose from yaml file for localization
        init_y: 1.305790
        init_z: 0.002673
        init_roll: 0.0
        init_pitch: 0.0
        init_yaw: -1.150664 
```
## Test Sample 

```
ros2 bag play cic_office_stopping.db3 --start-offset 50  
ros2 launch super_odometry arize_slam.launch.py 

```
## Load the start_pose.txt (below are format samples) 
```  
timespan x        y        z      roll     pitch     yaw
50s 13.983960 1.305790 0.002673   0.00     0.0 -1.150664
```
## Put the start_pose.txt same directory of groundtruth map (pointcloud_local.pcd)

## Test Bag file
https://drive.google.com/drive/u/0/folders/1R8Tx8nLDC184gjUaMPZjTiklIhsH7RLV
