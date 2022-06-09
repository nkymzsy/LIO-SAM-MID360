# LIO-SAM-MID360

本代码为LIO-SAM适配Livox MID360版本。

## Dependency

- 与LIO-SAM有相同依赖 ( [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM/) ）
- 请安装LIVOX ROS包用于发布点云数据 ( [览沃 ROS 驱动程序](https://github.com/Livox-SDK/livox_ros_driver/) )

## Prepare lidar data

Livox MID360雷达数据请使用览沃自定义点云数据`CustomMsg`，即LIVOX ROS驱动请运行`livox_lidar_msg.launch`。

## Note
- 注意对齐Lidar与IMU的时间戳。
- 注意修改Lidai与IMU的外参。(param.yaml文件)
## TODO

  - [ ] 提取特征优化
  - [ ] 使用MID360自带的六轴IMU


## Acknowledgement

  - LIO-SAM is based on LOAM (J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time).
