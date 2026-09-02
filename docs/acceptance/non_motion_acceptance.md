# GO2 Scout 风格迁移：非运动验收记录

验收环境：Jetson `tegra-ubuntu`，仓库 `/home/nvidia/go2_mid360_nav`，分支 `codex/scout-teb-migration`，安全修复基线 `aa07d6078054127c80e4b9c0510c1f3b0a04e20f`。本记录只证明源码、构建、测试和 launch 图状态，不代表已完成传感器或实机运动验收。

## 1. 仓库状态

- 时间：`2026-09-02T11:09:35+08:00`
- 命令：`git status --short --branch`
- 退出码：`0`
- 摘要：分支为 `codex/scout-teb-migration`；仅保留历史未跟踪目录 `catkin_ws/src/livox_ros_driver2/`、`ego_ws/`、`third_party/`，本次未删除或修改这些目录。

## 2. 运行产物未被 Git 跟踪

- 时间：`2026-09-02T11:09:35+08:00`
- 命令：`git ls-files | grep -E '(^|/)(build|devel|maps|bags|datasets|logs)/' || true`
- 退出码：`0`
- 摘要：无输出；build、devel、地图、bag、数据集和日志目录均未纳入 Git。

## 3. 已跟踪大文件检查

- 时间：`2026-09-02T11:09:35+08:00`
- 命令：`git ls-files -z | xargs -0 -r du -b | sort -nr | head -n 20`
- 退出码：`0`
- 摘要：最大文件主要是 `src/FAST_LIO` 与 `vendor/FAST_LIO` 的重复上游 GIF/PDF，以及 Unitree SDK2 的 aarch64/x86_64 静态库。最大单文件约 52.35 MB。它们是已知的历史依赖/演示资产，未在本次迁移中擅自删除，后续需维护者确认是否用 Git LFS、外部依赖或精简上游文档。

## 4. 自动测试

- 时间：`2026-09-02T11:51:50+08:00`
- 命令：`python3 -m pytest -q tests`
- 退出码：`0`
- 摘要：`39 passed in 2.83s`。

覆盖范围包括仓库边界、GO2 接口、建图、NDT 定位、move_base+TEB、禁横移、安全看门狗、非有限数据闭锁、定位健康超时、旧启动入口隔离和统一启动入口契约。

## 5. Catkin 编译

- 干净编译：安全修复后，在删除且仅删除仓库内生成目录 `build/`、`devel/` 后执行 `catkin_make -j1`，13 个包从零完成至 `100%`，退出码 `0`。最后两项有限性与周期刷新修复随后单独重新编译成功。
- 全工作空间复验时间：`2026-09-02T11:51:54+08:00`
- 复验命令：`catkin_make -j1`
- 复验退出码：`0`
- 摘要：13 个 Catkin 包完成配置，最终 `fastlio_mapping` 和 `go2_cloud_adapter_node` 均构建到 `100%`。PCL/VTK 的可选功能警告和上游 deprecated/unused-variable 警告未导致失败。

## 6. Launch 图契约

- 时间：`2026-09-02T11:51:56+08:00`
- 命令：`python3 scripts/validate_launch_contracts.py`
- 退出码：`0`
- 摘要：验证器展开完整 ROS launch 后按 package/type/name 检查。建图入口包含 `laserMapping`、`go2_pointcloud_mapper`；定位入口包含 `laserMapping`、`fast_lio_localization`、`go2_localization_guard`；导航入口包含完整定位链、`move_base` 和 `go2_cmd_watchdog`。三个入口均无活动 EGO 节点，导航入口不包含真实 Unitree SDK 桥接。

## 7. 安全代码复审

- 两轮审查发现并修复：NaN/Inf 命令处理、定位健康永久锁存、NDT 收敛/质量门缺失、初始定位健康计数、零消息速度误判、旧 Scout 启动旁路、非有限 odom/TF 传播，以及 NDT 刷新周期与健康期限不一致。
- 最终实现对全部 Twist 分量、odom、定位姿态、NDT 结果和派生 TF 做有限性检查；定位健康在 guard、watchdog 和真实 SDK 桥接间使用超时心跳；NDT 最长每 1 秒尝试刷新，健康成功年龄上限为 2 秒。
- 最终复审结论：没有剩余 Critical/Important 问题，可进行非运动交接；不代表批准实机运动。

## 8. 当前结论

代码迁移达到非运动验收条件：统一 Catkin 活跃源码树可从零构建，39 项测试通过，三套统一 launch 图可解析，TEB 禁止横移，速度命令经过定位门控和看门狗，真实 SDK 与导航入口分离。

尚未完成、必须现场监护的步骤：

1. 启动传感器和定位。
2. 设置初始位姿并确认地图/点云重合。
3. 确认 `/localization/ok` 为 true，且每条 TF 边只有一个发布者。
4. 在真实 SDK 桥接关闭时启动 move_base/TEB。
5. 检查 `/cmd_vel_nav` 的限速和 `linear.y == 0`。
6. 以后在实体急停、足够净空和现场监护下，才单独启动并人工使能真实 SDK 桥接。

本次验收到此停止：没有删除历史目录，没有启动真实 SDK，没有向机器人发送运动命令。
