# WVCSC localization config

This directory vendors Autoware localization parameters and applies a small set of WVCSC single-lidar overrides:

- NDT input pointcloud defaults to `/sensing/lidar/pointcloud_raw`
- IMU is bridged from `/sensing/imu/tamagawa/imu_raw` to `/sensing/imu/imu_data`
- `required_distance` is reduced to `5.0 m` for short-range small vehicle operation
- voxel downsampling is tightened to `1.0 m`
- random downsample count is raised to `3000`

These values are starting points for low-speed indoor/outdoor WVCSC testing and should be refined from bag data.
