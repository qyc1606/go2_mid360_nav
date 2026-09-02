# GO2 Scout 风格导航开发文档 V1

## 1. 目标与边界

本分支把 GO2 工程整理为与 Scout Mini 相同的工作层次：传感器、里程计、全局定位、导航规划、安全门和底层执行相互分离。算法链采用 Mid-360 + FAST-LIO + NDT + move_base + TEB；机器人外形、速度限制、雷达外参和 Unitree SDK 控制仍使用 GO2 自己的实现。

当前版本只完成非运动验收。真实 SDK 桥接不属于统一导航入口，不能因为启动导航而自动驱动机器狗。

## 2. 数据链

```text
/livox/lidar + /livox/imu
        ↓
FAST-LIO → /odom_nav + /cloud_registered_base
        ↓
NDT + 静态地图 → 质量门 → /localization + map→odom + /localization/ok
        ↓
move_base + TEB → /cmd_vel_nav
        ↓
go2_cmd_watchdog → /cmd_vel_safe
        ↓
真实 Unitree SDK 桥接（独立启动、默认禁用、需人工使能）
```

TEB 按非全向底盘配置，`max_vel_y=0`。首版限制为 `|vx|≤0.20 m/s`、`vy=0`、`|wz|≤0.30 rad/s`，命令超时为 0.50 秒。定位质量每 1 秒尝试用 NDT 刷新；guard、watchdog 和真实 SDK 桥接均对定位健康心跳超时闭锁。GO2 规划 footprint 暂按 0.70 m × 0.31 m；实机自主运动前必须连同腿部、防护件和载荷重新测量最大轮廓。

## 3. 活跃包职责

| 包 | 职责 |
|---|---|
| `livox_ros_driver2` | Mid-360 点云和 IMU 驱动 |
| `FAST_LIO` | 激光惯性里程计和建图 |
| `go2_pose_adapter`、`go2_cloud_adapter`、`go2_tf_manager` | 统一 GO2 导航话题与 TF |
| `go2_pointcloud_mapper`、`go2_map_tools` | 点云地图生成和导航栅格转换 |
| `fast_lio_localization` | 基于 NDT 的全局重定位，发布 `map→odom` |
| `go2_localization_guard` | 对定位新鲜度与跳变做状态判定 |
| `go2_navigation` | move_base、GlobalPlanner、TEB 和 costmap 参数 |
| `go2_cmd_watchdog` | 限速、禁横移、定位门控和命令超时停车 |
| `go2_sdk_bridge` | Unitree SDK2 执行层；真实节点必须独立启动和人工使能 |
| `go2_system_bringup` | 建图、定位和导航的统一入口 |

## 4. 统一入口

```bash
roslaunch go2_system_bringup go2_mapping.launch map_name:=site01
roslaunch go2_system_bringup go2_localization.launch map_name:=site01
roslaunch go2_system_bringup go2_navigation.launch map_name:=site01
```

三个入口都可用 `start_lidar:=false` 做 launch 图检查。导航入口包含定位前提、地图服务、move_base/TEB 和看门狗，但不包含真实 SDK 节点。

## 5. 地图约定

地图目录不进入 Git，默认位于 `~/go2_mid360_nav/maps/<map_name>/`。建图时 FAST-LIO 自带 PCD 自动保存被关闭，由 `go2_pointcloud_mapper` 输出 `public_map.pcd`，再生成供 NDT 和 move_base 使用的地图文件。地图、rosbag、日志和数据集属于运行数据，应备份到 NVMe 或外部存储，而不是提交到源码仓库。

## 6. 仓库边界

活跃 Catkin 源码入口是 `src/`。历史目录和第三方副本目前仍保留，以便师兄核对修改来源；本次迁移没有删除 `catkin_ws`、`ego_ws`、`localization_ws`、`vendor` 或 `third_party`。仓库内仍存在 FAST-LIO 等历史重复副本和大体积上游演示文件，后续应由项目维护者确认后再做去重，不能仅凭目录名称删除。

完整迁移前备份仍位于 Jetson NVMe：

```text
/media/nvidia/系统/go2_full_backups/go2_mid360_nav_full_20260831_1745.tar
```

## 7. 修改和验收原则

- GO2 参数集中在 `go2_system_bringup/config`、导航配置和各安全包中，不把 Scout 的底盘参数直接照搬。
- 新功能先写纯逻辑契约测试，再编译 ROS 包。
- 提交前运行 `python3 -m pytest -q tests`、`catkin_make -j1` 和 `python3 scripts/validate_launch_contracts.py`。
- 静态验收不能替代传感器、地图重合、TF 唯一发布者和现场急停条件下的实机验收。
