# GO2 Scout-Style TEB Migration Design

## Objective

Restructure the independent `qyc1606/go2_mid360_nav` repository to follow the same engineering layout and operating workflow as the Scout Mini project, while retaining GO2-specific hardware interfaces, calibration, geometry, safety limits, and Unitree SDK2 control.

The active navigation chain will replace EGO-Planner with ROS Navigation `move_base` and `teb_local_planner`. The first real-robot release will be non-holonomic: lateral velocity remains disabled even though GO2 hardware can move sideways.

## Fixed Decisions

- Keep GO2 in its independent repository during migration.
- Use one Catkin workspace with all active ROS packages under one `src/` tree.
- Match the Scout Mini package responsibilities and three operating workflows: mapping, localization, and navigation.
- Use FAST-LIO for local odometry and registered point clouds.
- Use the Scout-style NDT-OMP global localization path and enforce a single `map -> odom` publisher.
- Use `move_base` with `GlobalPlanner` and `teb_local_planner`.
- Remap planner output to `/cmd_vel_nav`; retain the existing GO2 watchdog and Unitree SDK2 bridge downstream.
- Set `max_vel_y: 0.0` and `acc_lim_y: 0.0` for the first release.
- Keep maps, bags, datasets, logs, build products, credentials, and machine-local backups outside Git.
- Do not modify or delete the verified full backup on the NVMe drive.

## Target Repository Layout

```text
go2_mid360_nav/
├── README.md
├── AGENTS.md
├── docs/
├── maps/                         # Runtime data; ignored by Git
├── scripts/                      # Build, setup, recording, and validation tools
└── src/
    ├── FAST_LIO/
    ├── Livox-SDK2/
    ├── livox_ros_driver2/
    ├── fast_lio_localization/    # NDT map loader and global localization
    ├── go2_sdk_bridge/
    ├── go2_system_bringup/
    ├── go2_tf_manager/
    ├── go2_pose_adapter/
    ├── go2_cloud_adapter/
    ├── go2_pointcloud_mapper/
    ├── go2_map_tools/
    ├── go2_navigation/
    ├── go2_cmd_watchdog/
    └── go2_localization_guard/
```

`build/`, `devel/`, and `install/` are generated beside `src/` and remain ignored. Third-party source is stored without nested `.git` directories, with upstream URL and pinned revision recorded in documentation.

## Package Responsibilities

### Hardware and drivers

- `livox_ros_driver2` and `Livox-SDK2` provide Mid-360 point cloud and IMU input.
- `go2_sdk_bridge` is the only package allowed to call Unitree SDK2 motion APIs.
- GO2 network interface names, SDK paths, and hardware identifiers are configuration values rather than hard-coded source paths.

### Localization and frame adaptation

- `FAST_LIO` publishes local odometry and registered point clouds.
- `go2_pose_adapter` converts the calibrated LiDAR/body pose into public planar `/odom_nav` and publishes `odom -> base_footprint -> base_link`.
- `go2_cloud_adapter` publishes registered clouds in the frame required by mapping and localization.
- `fast_lio_localization` loads the static PCD, performs NDT-OMP registration, and is the only owner of `map -> odom`.
- `go2_tf_manager` is the only owner of static robot and sensor transforms.
- `go2_localization_guard` evaluates localization health and publishes `/localization/ok`.

### Mapping and map assets

- `go2_pointcloud_mapper` consumes registered FAST-LIO output and generates the deliverable static PCD; FAST-LIO native PCD saving remains disabled.
- `go2_map_tools` converts and finalizes PCD, PGM, and YAML assets using a named-map directory.
- Active map paths are launch arguments. No C++ source contains `/home/nvidia/go2_mid360_nav` paths.

### Navigation and safety

- `go2_navigation` owns `move_base`, `GlobalPlanner`, TEB configuration, costmaps, footprint, launch files, and navigation validation tools.
- Global costmap uses `map`; local costmap uses `odom`; robot frame is `base_link`.
- TEB consumes `/odom_nav` and publishes commands remapped to `/cmd_vel_nav`.
- First-release limits disable lateral motion. Initial speed and acceleration limits are conservative and must not exceed the existing watchdog and SDK bridge limits.
- `/cmd_vel_nav -> go2_cmd_watchdog -> /cmd_vel_safe -> go2_sdk_bridge` remains the mandatory command path.
- Loss of localization, stale commands, invalid numeric values, disabled bridge state, or expired command timeout results in a zero command and StopMove behavior.

## Runtime Workflows

### Mapping

```text
Mid-360 -> FAST-LIO -> GO2 frame/cloud adapters -> pointcloud mapper -> named PCD map
```

The mapping launch starts only the required sensor, estimation, TF, pose, and map-generation nodes. It does not start `move_base` or the real motion bridge.

### Localization

```text
Mid-360 -> FAST-LIO local odom -> GO2 adapters
static PCD + registered cloud -> NDT localization -> map -> odom
```

The localization launch also publishes the 2D occupancy map required by navigation. Initial pose is supplied explicitly, and navigation cannot start until localization health is true.

### Navigation

```text
map + map->odom + odom->base_link + obstacle cloud
  -> move_base + GlobalPlanner + TEB
  -> /cmd_vel_nav
  -> watchdog
  -> /cmd_vel_safe
  -> Unitree SDK2
```

The real SDK bridge starts disabled. Enabling it is a separate operator action after visualization and dry-run checks pass.

## Migration Strategy

1. Work only on branch `codex/scout-teb-migration`; keep `main` unchanged.
2. Build the new single-workspace `src/` tree alongside the old workspaces.
3. Copy and rename Scout architectural packages, then replace all Scout hardware topics, frames, dimensions, and controls with GO2 equivalents.
4. Port only the active GO2-specific implementations needed by the new pipeline.
5. Keep EGO-Planner and the old multi-workspace tree out of the new active build. Their original files remain recoverable from the verified full backup and pre-migration checkout until acceptance.
6. Compile and validate each layer independently before enabling the next layer.
7. Remove old workspaces only after the new mapping, localization, dry-run navigation, and low-speed real-robot acceptance tests all pass, and only with explicit user approval.

## Validation and Acceptance

Validation proceeds without commanding motion until the final stage:

1. Repository hygiene: no nested Git repositories, generated files, maps, bags, secrets, or files over GitHub limits.
2. Clean `catkin_make -j1` build from an empty `build/` and `devel/`.
3. Static package, launch, parameter, and dependency checks.
4. Sensor and TF validation with unique publishers for every TF edge.
5. Mapping validation and deterministic map-finalization output.
6. NDT localization validation, including stable `map -> odom` and localization-loss behavior.
7. `move_base` and TEB dry run with the SDK bridge disabled; confirm `linear.y` is always zero.
8. Watchdog tests for timeout, localization loss, NaN/Inf rejection, and limit clamping.
9. Low-speed real-robot test in open space with an operator ready to stop the robot.

## Rollback

- Switch back to `main` to return to the pre-migration tracked state.
- The full Jetson project, nested repositories, maps, data, and local modifications remain available in `/media/nvidia/系统/go2_full_backups/go2_mid360_nav_full_20260831_1745.tar` with its verified SHA-256 file.
- The Scout Mini repository remains an independent read-only reference at `/media/nvidia/系统/reference_projects/AADCL_UAV_UGV`.
