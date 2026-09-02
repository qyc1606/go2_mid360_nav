# GO2 建图、定位与实机导航完整使用手册 V1

> 本手册用于 Jetson 上 GO2 + Mid-360 的现场测试。当前架构为 FAST-LIO 建图、NDT 定位、move_base + TEB 规划、Unitree SDK2 执行。所有命令默认在 Jetson `nvidia@192.168.50.100` 上运行。

## 0. 实机测试边界

开始前必须同时满足：

- GO2 周围有足够净空，首次运动只做低速、短距离测试；
- 一人操作电脑，一人现场看护机器狗；
- 原厂遥控器和实体急停方式可立即使用，并先验证能让 GO2 停止；
- GO2 已正常站立并处于可接受 SportClient 速度命令的状态；
- 不在桌面、台阶、斜坡、玻璃门旁或人员密集区域测试；
- 建图、单独定位和统一导航不能重复启动；同一时刻只能有一套 FAST-LIO 和一套 `map→odom` 发布者。

真实 SDK2 节点启动后仍是禁用状态。只有执行本手册的人工使能命令后，GO2 才可能运动。

## 1. 登录、确认代码和编译

### 1.1 从电脑登录 Jetson

```bash
ssh nvidia@192.168.50.100
```

### 1.2 确认使用新工程

```bash
cd ~/go2_mid360_nav
git branch --show-current
git status --short --branch
```

分支应为：

```text
codex/scout-teb-migration
```

只编译 `~/go2_mid360_nav/src/`。不要进入旧目录 `~/go2_mid360_nav/catkin_ws/` 编译。

### 1.3 编译和静态检查

```bash
cd ~/go2_mid360_nav
source /opt/ros/noetic/setup.bash
catkin_make -j1
source devel/setup.bash
python3 -m pytest -q tests
python3 scripts/validate_launch_contracts.py
```

预期自动测试全部通过，并显示三个 canonical launch contracts passed。失败时不要继续实机测试。

### 1.4 检查 GO2 控制网口

```bash
ip -br addr show eth1
ip neigh show dev eth1
```

本机当前 GO2 网口是 `eth1`，地址应在 `192.168.123.0/24`；当前 Jetson 配置为 `192.168.123.199/24`。如果现场接线使用了别的接口，后面的 `network_interface:=eth1` 必须一并修改。

### 1.5 终端 0：单独启动 ROS Master

在一个终端启动 `roscore`，并在整个建图、定位和导航测试期间保持该终端运行：

```bash
source /opt/ros/noetic/setup.bash
roscore
```

等待出现 `started core service [/rosout]`后，再打开后续终端。`roslaunch` 在没有 ROS Master 时本可以自动启动一个，但实机多终端测试明确单独运行 `roscore` 更容易检查和停机。

## 2. 第一次测试：建立点云地图

建图时使用原厂遥控器让 GO2 缓慢行走，不启动本工程的真实 SDK2 桥接。

### 2.1 终端 1：启动建图

地图名示例使用 `site01`。同一场地后续定位和导航必须使用相同名字。

```bash
cd ~/go2_mid360_nav
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch go2_system_bringup go2_mapping.launch map_name:=site01
```

该命令启动 Mid-360、FAST-LIO、GO2 坐标适配和静态点云累积器。FAST-LIO 自带的大 PCD 自动保存已关闭，最终地图由 `go2_pointcloud_mapper` 保存。

### 2.2 终端 2：检查建图数据

```bash
cd ~/go2_mid360_nav
source devel/setup.bash
rosparam get /go2_pointcloud_mapper/input_cloud
rostopic echo -n 1 /cloud_registered_odom/header
rostopic echo -n 1 /odom_nav/header
rostopic hz /livox/lidar
rostopic hz /livox/imu
rostopic hz /odom_nav
rostopic hz /cloud_registered_base
rostopic hz /cloud_registered_odom
rostopic hz /go2/static_map_cloud
```

首条命令必须输出 `/cloud_registered_odom`，后两条 header 的 `frame_id` 都必须是 `odom`。如果出现 `Mapper frame mismatch`，立即停止建图，不要继续走。

其他频率命令每条观察数秒后按 `Ctrl+C` 结束当前观察，再执行下一条。应持续有频率输出，不能反复出现时间倒退、TF 错误或点云为空。`/cloud_registered_base` 应是 `base_link` 坐标系，供 RViz 和局部障碍检查；`/cloud_registered_odom` 应是 `odom` 坐标系，这才是 mapper 累积全局地图的输入。

可另外打开 RViz：

```bash
rviz
```

将 Fixed Frame 设为 `odom`，添加 PointCloud2 `/cloud_registered_base` 和 `/go2/static_map_cloud`。确认点云方向正确、地面基本水平，机器人走回原位置时地图能闭合。

### 2.3 建图操作要求

- 低速、平稳移动，避免急转、跳跃和碰撞；
- 从不同方向覆盖门口、走廊、拐角和障碍物边缘；
- 动态人员和推车离开后，再次观察原区域；
- 不要长时间停留在一个位置，也不要在雷达被人遮挡时保存地图。

### 2.4 保存地图，随后停止建图

当前 mapper 不自动保存，必须先在终端 2 执行：

```bash
rosservice call /go2_pointcloud_mapper/save_map "{}"
ls -lh ~/go2_mid360_nav/maps/site01/public_map.pcd
```

必须看到服务返回 `success: True`，并确认 `public_map.pcd` 存在且大小不是 0。

然后回到终端 1，按一次 `Ctrl+C`，等待节点正常退出。不要使用 `kill -9`，也不要直接给 Jetson 断电。

### 2.5 生成 NDT 和 move_base 使用的地图资产

在所有建图节点退出后执行：

```bash
cd ~/go2_mid360_nav
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 scripts/finalize_map.py site01
ls -lh maps/site01/
```

目录至少应包含：

```text
public_map.pcd
map.pgm
map.yaml
map_raw.pgm
map_raw.yaml
```

`public_map.pcd` 供 NDT 定位使用，`map_raw.yaml` 供 move_base 全局 costmap 使用。缺少任何一个文件都不要进入导航测试。

`finalize_map.py` 会分别使用 `corridor_raw.yaml` 生成不预膨胀的 `map_raw.pgm`，以及使用 `corridor_nav.yaml` 生成带可视化障碍膨胀的 `map.pgm`。move_base 加载的是 `map_raw.yaml`，障碍安全距离由 costmap 在运行时管理，不应在 raw 地图中重复膨胀。

## 3. 第二次测试：单独验证 NDT 定位

### 3.1 终端 1：启动定位

```bash
cd ~/go2_mid360_nav
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch go2_system_bringup go2_localization.launch map_name:=site01
```

### 3.2 终端 2：打开 RViz 并设置初始位姿

```bash
cd ~/go2_mid360_nav
source devel/setup.bash
rviz
```

在 RViz 中：

1. Fixed Frame 设为 `map`；
2. 添加 Map `/map_2d`；
3. 添加 PointCloud2 `/map_cloud`；
4. 添加 PointCloud2 `/cloud_registered_base`；
5. 使用 `2D Pose Estimate` 在地图上给出 GO2 当前大致位置和朝向。

定位节点启动后会先由同一个 TF 发布者给出临时单位变换 `map→odom`，以便 RViz 能选择 `map` 并发送初始位姿。此时 `/localization/ok` 仍必须是 `false`，临时 TF 不代表定位成功。NDT 首次匹配成功后，该节点会把它更新为真实的 `map→odom`。

初值必须尽量准确。等待 NDT 成功后，历史地图与实时点云应稳定重合。

### 3.3 终端 3：定位验收

分别执行：

```bash
rostopic echo /localization/ok
rosrun tf tf_echo map odom
rosrun tf tf_echo map base_link
rostopic hz /localization
```

必须满足：

- `/localization/ok` 稳定为 `data: true`；
- `map→odom→base_link→lidar_link` 连续；
- 点云与地图重合，没有明显旋转、镜像或固定偏移；
- 机器狗静止时定位不会在约 2 秒后变为 LOST；
- 每条 TF 边只有一个发布者。

定位不通过时，不得启动 SDK2。完成单独定位检查后，在终端 1 按 `Ctrl+C` 停止定位，再进入下一阶段，避免与统一导航入口重复启动。

## 4. 第三次测试：move_base + TEB 规划空跑

本阶段会生成速度命令，但真实 SDK2 尚未启动，GO2 不应被本工程驱动。

### 4.1 终端 1：启动统一导航

```bash
cd ~/go2_mid360_nav
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch go2_system_bringup go2_navigation.launch map_name:=site01
```

该入口一次性启动 Mid-360、FAST-LIO、NDT、地图服务、move_base、TEB 和安全看门狗，不会自动启动 `go2_sdk_bridge_real`。

### 4.2 终端 2：打开 RViz并重新设置初始位姿

```bash
cd ~/go2_mid360_nav
source devel/setup.bash
rviz
```

Fixed Frame 设为 `map`，添加 `/map_2d`、实时点云、Global Path、Local Path 和 TF。再次使用 `2D Pose Estimate`，直到 `/localization/ok` 为 true。

### 4.3 发送测试目标并检查规划速度

先在 RViz 中设置一个很近、没有障碍物的 `2D Nav Goal`。此时 GO2 不应运动，但应能看到全局路径、局部轨迹和 `/cmd_vel_nav`。

终端 3 执行：

```bash
cd ~/go2_mid360_nav
source devel/setup.bash
rostopic echo /cmd_vel_nav
python3 scripts/check_cmd_vel.py --duration 10
```

检查脚本必须显示 PASS，并且至少收到一条命令。要求：

```text
|linear.x| <= 0.20 m/s
linear.y == 0
|angular.z| <= 0.30 rad/s
全部 Twist 分量均为有限数值
```

取消当前空跑目标，避免 SDK2 使能后立即执行旧目标：

```bash
rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID "{}"
rostopic echo -n 5 /cmd_vel_safe
```

确认 `/cmd_vel_safe` 已回到零速。

## 5. 第四次测试：启动真实 SDK2 并进行低速实机导航

只有第 2、3、4 节全部通过后才能执行本节。现场看护人员必须站在可立即操作遥控器/急停的位置。

### 5.1 先准备禁用命令

在电脑上提前复制好下面这条命令，任何异常立即执行：

```bash
rosservice call /go2_sdk_bridge_real/enable "data: false"
```

### 5.2 终端 4：启动真实 SDK2，但保持禁用

```bash
cd ~/go2_mid360_nav
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch go2_sdk_bridge sdk_bridge_real.launch network_interface:=eth1
```

启动日志应包含：

```text
REAL GO2 SDK bridge started DISABLED
```

这一步已经初始化 Unitree SDK2 和 SportClient，但尚未允许 Move 命令。检查节点和输入：

```bash
rosnode info /go2_sdk_bridge_real
rostopic hz /cmd_vel_safe
rostopic echo -n 5 /localization/ok
```

### 5.3 人工使能 SDK2

确认 GO2 正常站立、周围清空、遥控急停可用、`/localization/ok` 为 true、`/cmd_vel_safe` 为零后执行：

```bash
rosservice call /go2_sdk_bridge_real/enable "data: true"
```

预期返回：

```text
success: True
message: "REAL GO2 SDK bridge ENABLED."
```

如果返回 false，不要绕过检查。根据 message 检查定位是否为 true、速度话题是否存在且时间新鲜。

### 5.4 第一次实机目标

推荐按以下顺序逐项测试，每次只发一个目标：

1. 原地小角度转向；
2. 前方约 0.3～0.5 米的直线目标；
3. 前方约 1 米、无障碍物的目标；
4. 最后才测试绕开单个静态障碍物。

在 RViz 使用 `2D Nav Goal` 发送目标。终端 3 同时运行：

```bash
python3 scripts/check_cmd_vel.py --duration 60
```

现场观察 GO2 实际运动方向是否与 RViz 目标一致。出现反向运动、侧向运动、抖动、突然加速、地图错位、路径穿障碍或定位变为 false 时，立即执行禁用命令。

### 5.5 随时停止与重新使能

软件停止：

```bash
rosservice call /go2_sdk_bridge_real/enable "data: false"
rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID "{}"
```

禁用后 SDK2 桥接会发送 StopMove。定位丢失、定位心跳超时、速度命令超时、NaN/Inf 或节点退出也会自动闭锁，但不能依赖软件保护替代遥控器/实体急停。

重新使能前必须重新确认定位、TF、地图重合和零速，再执行：

```bash
rosservice call /go2_sdk_bridge_real/enable "data: true"
```

## 6. 正常停止顺序

严格按以下顺序停止：

1. 禁用真实 SDK2：

   ```bash
   rosservice call /go2_sdk_bridge_real/enable "data: false"
   ```

2. 取消 move_base 目标：

   ```bash
   rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID "{}"
   ```

3. 确认 `/cmd_vel_safe` 连续为零：

   ```bash
   rostopic echo -n 5 /cmd_vel_safe
   ```

4. 在 SDK2 终端按一次 `Ctrl+C`，等待 `StopMove sent` 日志；
5. 在统一导航终端按一次 `Ctrl+C`；
6. 关闭 RViz 和检查终端；
7. 回到终端 0，按 `Ctrl+C` 停止 `roscore`；
8. 最后再关闭 GO2 和 Jetson 电源。

建图模式停止前必须先调用地图保存服务；导航模式不需要保存地图。

## 7. 使用已有地图直接测试

如果 `maps/site01/` 已有五个完整地图文件，可跳过建图，直接从第 3 节定位开始。先检查：

```bash
cd ~/go2_mid360_nav
ls -lh maps/site01/public_map.pcd \
       maps/site01/map.pgm \
       maps/site01/map.yaml \
       maps/site01/map_raw.pgm \
       maps/site01/map_raw.yaml
```

不要把其他场地地图改名后直接使用。

## 8. 快速排错

| 现象 | 首先检查 |
|---|---|
| 无 `/livox/lidar` | Mid-360 供电、网线、雷达 IP、驱动日志 |
| 无 `/odom_nav` | IMU 初始化、FAST-LIO 日志、点云/IMU时间戳 |
| 点云方向错误 | `base_link→lidar_link` 外参，不要靠 RViz 旋转补偿 |
| 地图没有保存 | 是否调用 `/go2_pointcloud_mapper/save_map`，服务是否返回 true |
| `finalize_map.py` 报缺少 PCD | 地图名是否一致，`public_map.pcd` 是否存在 |
| NDT 不收敛 | 地图名、初始位姿、实时点云、雷达外参 |
| RViz 的 Fixed Frame 没有 `map` | `roscore`、`fast_lio_localization` 节点和临时 `map→odom` 是否正常发布 |
| `/localization/ok` 为 false | NDT score、刷新周期、TF、点云地图重合 |
| 无全局路径 | `/map_2d`、目标是否在自由区域、`map→base_link` |
| 地图大部分是灰色且无路径 | 灰色为未知区，不要直接开启 `allow_unknown`；重新运行最新 `finalize_map.py`，确认 `map_raw` 使用 raw 配置且未重复膨胀 |
| 有路径但无 `/cmd_vel_nav` | TEB 状态、local costmap、目标是否已取消 |
| SDK2 启动失败 | `eth1` 地址、GO2 网络、Unitree SDK2 日志 |
| SDK2 使能返回 false | `/localization/ok` 和 `/cmd_vel_safe` 是否存在且新鲜 |
| GO2 不动但使能成功 | GO2 Sport 模式、急停/遥控状态、`/cmd_vel_safe` 是否非零 |
| GO2 运动异常 | 立即禁用 SDK2并取消目标，检查坐标方向、外参和速度符号 |

## 9. 关键目录

```text
~/go2_mid360_nav/src/                 当前新源码
~/go2_mid360_nav/maps/               地图数据，不进入 Git
~/go2_mid360_nav/docs/               使用与开发文档
~/go2_mid360_nav/catkin_ws/           历史旧工作空间，不参与当前编译
```

完整接口和 TF 基准见 `docs/GO2_话题_启动文件_TF_检查表_V1.md`。首次实机测试建议逐步记录每条命令的终端输出，发生异常时保留完整日志，不要立即改多个参数。
