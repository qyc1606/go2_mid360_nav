# GO2 导航使用手册 V1

## 1. 编译与静态检查

```bash
cd ~/go2_mid360_nav
source /opt/ros/noetic/setup.bash
catkin_make -j1
source devel/setup.bash
python3 -m pytest -q tests
python3 scripts/validate_launch_contracts.py
```

`validate_launch_contracts.py` 只展开 launch 图，不启动 ROS 节点。任何失败都应先处理，不得进入实机阶段。

## 2. 建图

```bash
roslaunch go2_system_bringup go2_mapping.launch map_name:=site01
```

建图期间观察 `/odom_nav`、`/cloud_registered_base` 和 TF 是否连续。地图默认写入 `~/go2_mid360_nav/maps/site01/`，不提交 Git。结束后按项目地图整理流程确认 PCD、二维导航地图和 YAML 文件成套存在。

## 3. 定位

```bash
roslaunch go2_system_bringup go2_localization.launch map_name:=site01
```

在 RViz 设置初始位姿，检查地图与实时点云重合。必须确认 `/localization/ok` 稳定为 `true`，并确认 `map→odom→base_link→lidar_link` 每条 TF 边只有一个发布者。

## 4. 非运动导航检查

真实 SDK 桥接保持关闭，启动统一导航入口：

```bash
roslaunch go2_system_bringup go2_navigation.launch map_name:=site01
```

发送目标点后只观察规划、costmap 和 `/cmd_vel_nav`，不要启动真实 SDK。在目标有效且应持续产生规划命令时，用只读脚本检查命令：

```bash
python3 scripts/check_cmd_vel.py --duration 10
```

检查必须收到至少一条命令，并满足 `linear.y == 0`、`|linear.x|≤0.20`、`|angular.z|≤0.30`，全部六个 Twist 分量均不得出现 NaN/Inf。

## 5. 强制安全顺序

1. 启动传感器和定位。
2. 设置初始位姿，确认地图与点云重合。
3. 确认 `/localization/ok` 为 `true`，且每条 TF 边只有一个发布者。
4. 在真实 SDK 桥接关闭的情况下启动 move_base/TEB。
5. 确认 `/cmd_vel_nav` 未超限并且 `linear.y == 0`。
6. 只能在后续有人现场监护的测试中，单独启动并手动使能真实 SDK 桥接。

此顺序不得跳步。真实 SDK 节点即使启动也默认禁用，并会检查定位状态、命令新鲜度和速度限制；这些软件保护不能替代实体急停、架空测试和现场监护。

## 6. 停止与异常处理

- 定位丢失、TF 冲突、点云错位或速度检查失败时，停止导航排查，不使能真实桥接。
- `/cmd_vel_nav` 超时后看门狗应输出零速；仍需确认底盘实际停止。
- 地图文件缺失时不要临时改成其他地图名，应重新确认地图目录内容。
- 不在运行时删除建图数据、工作空间或历史依赖目录。
