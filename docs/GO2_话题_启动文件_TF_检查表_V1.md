# GO2 话题、启动文件与 TF 检查表 V1

## 启动文件

| 场景 | 命令 | 必须出现的核心节点 | 真实 SDK |
|---|---|---|---|
| 建图 | `roslaunch go2_system_bringup go2_mapping.launch map_name:=site01` | `laserMapping`、`go2_pointcloud_mapper` | 不启动 |
| 定位 | `roslaunch go2_system_bringup go2_localization.launch map_name:=site01` | `laserMapping`、`fast_lio_localization`、`go2_localization_guard` | 不启动 |
| 导航 | `roslaunch go2_system_bringup go2_navigation.launch map_name:=site01` | 上述定位节点、`move_base`、`go2_cmd_watchdog` | 不启动 |

静态检查：

```bash
python3 scripts/validate_launch_contracts.py
```

## 关键话题

| 话题 | 类型/方向 | 用途 | 检查重点 |
|---|---|---|---|
| `/livox/lidar` | 输入 | Mid-360 点云 | 频率、时间戳、无持续丢包 |
| `/livox/imu` | 输入 | Mid-360 IMU | 频率和时间同步 |
| `/odom_nav` | FAST-LIO → 导航 | 局部里程计 | 连续、无突跳、frame 正确 |
| `/cloud_registered_base` | FAST-LIO/适配层 → 定位与 costmap | base_link 坐标点云 | 与机器人和地图方向一致 |
| `/map_2d` | map_server → global costmap | 二维导航地图 | 地图名、原点、分辨率正确 |
| `/localization` | NDT → guard | 全局定位姿态 | 与点云重合、无异常跳变 |
| `/localization/ok` | guard → 安全层 | 定位健康门 | 导航前稳定为 true |
| `/cmd_vel_nav` | move_base/TEB → watchdog | 未执行的规划速度 | `vy=0` 且不超限 |
| `/cmd_vel_safe` | watchdog → SDK bridge | 安全过滤后的速度 | 定位失败或超时时为零 |

## TF 树

目标主链：

```text
map → odom → base_link → lidar_link
```

- `map→odom`：只由 NDT 全局定位发布。
- `odom→base_link`：只由里程计/GO2 TF 适配层发布。
- `base_link→lidar_link`：静态外参，来源为 GO2 Mid-360 标定。
- 不允许 FAST-LIO、NDT 和旧 EGO 启动文件同时重复发布同一条边。

检查命令示例：

```bash
rosrun tf tf_echo map odom
rosrun tf tf_echo odom base_link
rosrun tf tf_echo base_link lidar_link
rostopic echo -n 5 /localization/ok
rostopic hz /odom_nav
rostopic hz /cloud_registered_base
```

## 导航前勾选项

- [ ] 使用的是预期地图目录和地图名。
- [ ] 地图与实时点云稳定重合。
- [ ] `map→odom→base_link→lidar_link` 连续且无重复发布者。
- [ ] `/localization/ok` 稳定为 `true`。
- [ ] TEB footprint 已按实机最大轮廓复测。
- [ ] `/cmd_vel_nav` 满足 `vx≤0.20`、`vy=0`、`|wz|≤0.30`。
- [ ] 真实 SDK 桥接没有随导航入口自动启动。
- [ ] 首次实机测试具备现场监护、实体急停和足够净空。

