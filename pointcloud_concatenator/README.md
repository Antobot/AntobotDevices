# pointcloud_concatenator

这是从 Autoware 点云拼接思路中拆出来的 ROS 2 独立包。它不依赖 Autoware
专属包，只依赖标准 ROS 2 点云、TF 相关包。

## 功能

- 订阅多路 `sensor_msgs/msg/PointCloud2`
- 按 `naive` 或 `advanced` 策略把输入点云分到同一时间组
- 第一帧进入时间组后启动 `timeout_sec` 定时器
- 所有 topic 到齐时立即拼接
- 超时时用已收到的 topic 拼接
- 通过 TF 将每路输入转换到统一 `output_frame`
- 输出标准 `sensor_msgs/msg/PointCloud2` 到 `~/output`

## 和 Autoware 原版的差异

- 去掉了 `autoware_sensing_msgs/ConcatenatedPointCloudInfo`
- 去掉了 `autoware_utils`、`managed_transform_buffer`、`autoware_point_types`
- 没有强制转换成 `XYZIRC` 字段布局
- 要求参与拼接的输入点云 `PointCloud2` 字段布局一致；不一致的 topic 会被跳过
- 暂未移植基于 twist/odom 的运动补偿

## 构建

把整个目录放进任意 ROS 2 workspace 的 `src` 下：

```bash
colcon build --packages-select pointcloud_concatenator
```

## 运行

```bash
ros2 launch pointcloud_concatenator concatenate_pointclouds.launch.py
```

或指定自己的参数文件：

```bash
ros2 launch pointcloud_concatenator concatenate_pointclouds.launch.py \
  param_file:=/path/to/concatenator.param.yaml
```

## 关键参数

- `input_topics`: 输入点云 topic 列表，至少两个
- `output_frame`: 拼接前统一转换到的目标坐标系
- `timeout_sec`: collector 收到第一帧后最多等待多久
- `matching_strategy.type`: `naive` 或 `advanced`
- `matching_strategy.lidar_timestamp_offsets`: advanced 模式下每个 topic 的时间 offset
- `matching_strategy.lidar_timestamp_noise_window`: advanced 模式下每个 topic 的时间噪声窗口

## 超时逻辑

每个 collector 表示一组等待拼接的点云。第一帧点云进入 collector 时启动 timer：

- 如果所有 `input_topics` 都到了，立即拼接并发布
- 如果 `timeout_sec` 到期还没收齐，则用已有点云拼接并发布
