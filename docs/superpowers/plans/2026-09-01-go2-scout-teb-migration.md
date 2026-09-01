# GO2 Scout-Style TEB Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Convert the independent GO2 repository into a single Catkin workspace matching the Scout Mini operating structure, with FAST-LIO mapping, NDT localization, and non-holonomic `move_base + TEB` navigation feeding the existing GO2 safety and Unitree SDK2 command chain.

**Architecture:** Active ROS packages live under one root `src/` tree. Scout packages supply the workflow and navigation structure; GO2 packages retain robot geometry, calibration, frame conversion, localization health, watchdog, and SDK2 motion control. Old workspaces remain outside the active build until the replacement passes acceptance.

**Tech Stack:** Ubuntu 20.04, ROS Noetic, Catkin, C++14, Python 3, FAST-LIO, PCL/NDT-OMP, ROS Navigation, GlobalPlanner, teb_local_planner, Livox Mid-360, Unitree SDK2.

**Spec:** `docs/superpowers/specs/2026-09-01-go2-scout-teb-migration-design.md`

## Global Constraints

- Work only on `codex/scout-teb-migration`; do not alter `main`.
- Build with `catkin_make -j1` on Jetson.
- Keep `/cmd_vel_nav -> go2_cmd_watchdog -> /cmd_vel_safe -> go2_sdk_bridge` unchanged.
- Set `max_vel_y: 0.0` and `acc_lim_y: 0.0` in TEB and every downstream limiter.
- `go2_tf_manager` owns static TF; NDT localization exclusively owns `map -> odom`; `go2_pose_adapter` owns `odom -> base_footprint -> base_link`.
- Keep FAST-LIO native PCD saving disabled; the point-cloud mapper produces deliverable maps.
- Do not add maps, bags, datasets, logs, build products, credentials, archives, nested `.git` directories, or files larger than 95 MiB to Git.
- Do not command real robot motion during Tasks 1-8.
- Do not remove legacy workspaces without explicit user approval after acceptance.

---

### Task 1: Dependency Baseline and Repository Contract Tests

**Files:**
- Create: `tests/test_repository_contract.py`
- Create: `docs/dependencies.md`
- Modify: `.gitignore`

**Interfaces:**
- Consumes: ROS Noetic installation and the existing repository.
- Produces: executable static checks that every later task must pass.

- [ ] **Step 1: Write the failing repository contract test**

```python
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_required_root_entries_exist():
    for path in ("src", "README.md", "docs", "scripts"):
        assert (ROOT / path).exists(), path


def test_runtime_artifacts_are_ignored():
    text = (ROOT / ".gitignore").read_text(encoding="utf-8")
    for rule in ("/build/", "/devel/", "/install/", "/maps/", "/bags/", "/datasets/", "*.bag", "*.pcd"):
        assert rule in text, rule


def test_no_nested_git_directories_in_active_source():
    src = ROOT / "src"
    assert not list(src.rglob(".git")) if src.exists() else True


def test_no_large_tracked_candidates():
    if not (ROOT / "src").exists():
        return
    oversized = [p for p in (ROOT / "src").rglob("*") if p.is_file() and p.stat().st_size > 95 * 1024 * 1024]
    assert not oversized, oversized
```

- [ ] **Step 2: Run the test and verify it fails because the new root workspace does not exist**

Run:

```bash
cd /home/nvidia/go2_mid360_nav
python3 -m pytest -q tests/test_repository_contract.py
```

Expected: FAIL on missing `src` and `README.md`.

- [ ] **Step 3: Install the available ROS navigation dependencies**

Run:

```bash
sudo apt-get update
sudo apt-get install -y python3-pytest ros-noetic-navigation ros-noetic-teb-local-planner
source /opt/ros/noetic/setup.bash
rospack find move_base
rospack find global_planner
rospack find teb_local_planner
rospack find costmap_2d
```

Expected: each `rospack find` prints a path under `/opt/ros/noetic/share`.

- [ ] **Step 4: Extend `.gitignore` and record exact dependency versions**

Add these exact root rules:

```gitignore
/build/
/devel/
/install/
/maps/
/bags/
/datasets/
/logs/
*.bag
*.pcd
*.pgm
*.zip
*.tar
*.tar.gz
**/.git/
**/__pycache__/
*.pyc
```

Create `docs/dependencies.md` with the output of:

```bash
dpkg-query -W -f='${Package} ${Version}\n' python3-pytest ros-noetic-navigation ros-noetic-teb-local-planner
git -C /media/nvidia/系统/reference_projects/AADCL_UAV_UGV rev-parse HEAD
git -C /home/nvidia/go2_mid360_nav/vendor/FAST_LIO rev-parse HEAD 2>/dev/null || true
```

- [ ] **Step 5: Commit the baseline**

```bash
git add .gitignore tests/test_repository_contract.py docs/dependencies.md
git commit -m "test: define GO2 workspace repository contract"
```

---

### Task 2: Create the Single Active Catkin Source Tree

**Files:**
- Create: `src/CMakeLists.txt`
- Create: `README.md`
- Create: `src/THIRD_PARTY.md`
- Create: active source copies under `src/`

**Interfaces:**
- Consumes: existing GO2 sources and the read-only Scout Mini repository.
- Produces: one Catkin-discoverable `src/` tree without nested repositories or generated data.

- [ ] **Step 1: Add a failing package-layout assertion**

Append to `tests/test_repository_contract.py`:

```python
REQUIRED_PACKAGES = {
    "FAST_LIO", "livox_ros_driver2", "fast_lio_localization",
    "go2_sdk_bridge", "go2_system_bringup", "go2_tf_manager",
    "go2_pose_adapter", "go2_cloud_adapter", "go2_pointcloud_mapper",
    "go2_map_tools", "go2_navigation", "go2_cmd_watchdog",
    "go2_localization_guard",
}


def test_active_packages_exist():
    present = {p.parent.name for p in (ROOT / "src").rglob("package.xml")}
    assert REQUIRED_PACKAGES <= present
```

- [ ] **Step 2: Verify the new test fails**

```bash
python3 -m pytest -q tests/test_repository_contract.py
```

Expected: FAIL with the missing package set.

- [ ] **Step 3: Initialize the root Catkin workspace and import clean source copies**

Run these exact source-copy operations, excluding nested Git metadata and runtime artifacts:

```bash
cd /home/nvidia/go2_mid360_nav
mkdir -p src
cd src && catkin_init_workspace && cd ..

rsync -a --exclude=.git --exclude=Log --exclude=PCD \
  vendor/FAST_LIO/ src/FAST_LIO/
rsync -a --exclude=.git \
  catkin_ws/src/livox_ros_driver2/ src/livox_ros_driver2/
rsync -a --exclude=.git \
  /media/nvidia/系统/reference_projects/AADCL_UAV_UGV/Scout_mini/src/fast_lio_localization/ \
  src/fast_lio_localization/
rsync -a --exclude=.git \
  /media/nvidia/系统/reference_projects/AADCL_UAV_UGV/Scout_mini/src/Livox-SDK2/ \
  src/Livox-SDK2/
```

Copy the remaining source using this exact mapping:

```text
catkin_ws/src/go2_sdk_bridge                         -> src/go2_sdk_bridge
catkin_ws/src/go2_cmd_watchdog                       -> src/go2_cmd_watchdog
catkin_ws/src/go2_localization_guard                  -> src/go2_localization_guard
catkin_ws/src/go2_map_tools                           -> src/go2_map_tools
catkin_ws/src/go2_base/go2_tf_manager                 -> src/go2_tf_manager
catkin_ws/src/go2_base/go2_pose_adapter               -> src/go2_pose_adapter
catkin_ws/src/go2_base/cloud_frame_adapter            -> src/go2_cloud_adapter
catkin_ws/src/go2_bringup                              -> src/go2_system_bringup
Scout_mini/src/scout_pointcloud_mapper                -> src/go2_pointcloud_mapper
Scout_mini/src/scout_navigation                       -> src/go2_navigation
third_party/unitree_sdk2                              -> src/Unitree_SDK2
```

Use `rsync -a --exclude=.git --exclude='*.save*' --exclude='*.swp' --exclude=__pycache__` for every mapping. For the four renamed Scout/current packages, replace package identifiers only within the copied target:

```bash
python3 - <<'PY'
from pathlib import Path

replacements = {
    "src/go2_cloud_adapter": ("cloud_frame_adapter", "go2_cloud_adapter"),
    "src/go2_system_bringup": ("go2_bringup", "go2_system_bringup"),
    "src/go2_pointcloud_mapper": ("scout_pointcloud_mapper", "go2_pointcloud_mapper"),
    "src/go2_navigation": ("scout_navigation", "go2_navigation"),
}
for root, (old, new) in replacements.items():
    for path in Path(root).rglob("*"):
        if path.is_file():
            try:
                text = path.read_text(encoding="utf-8")
            except UnicodeDecodeError:
                continue
            path.write_text(text.replace(old, new), encoding="utf-8")
PY
```

- [ ] **Step 4: Create source provenance documentation and root README**

`src/THIRD_PARTY.md` must list source URL, copied commit, local modifications, and license for FAST-LIO, Livox SDK2/driver, and NDT localization. `README.md` must show the three canonical commands:

```bash
roslaunch go2_system_bringup go2_mapping.launch map_name:=site01
roslaunch go2_system_bringup go2_localization.launch map_name:=site01
roslaunch go2_navigation navigation_teb.launch map_name:=site01
```

- [ ] **Step 5: Run repository tests and Catkin package discovery**

```bash
python3 -m pytest -q tests/test_repository_contract.py
source /opt/ros/noetic/setup.bash
catkin_make -j1
```

Expected: repository tests pass; Catkin configures the active package tree without duplicate package-name errors.

- [ ] **Step 6: Commit the source tree**

```bash
git add README.md src tests/test_repository_contract.py
git commit -m "refactor: create single GO2 Catkin source tree"
```

---

### Task 3: Port GO2 Frames, Odometry, Cloud, and Hardware Control

**Files:**
- Modify: `src/go2_tf_manager/**`
- Modify: `src/go2_pose_adapter/**`
- Modify: `src/go2_cloud_adapter/**`
- Modify: `src/go2_sdk_bridge/**`
- Create: `src/go2_system_bringup/config/go2_geometry.yaml`
- Create: `src/go2_system_bringup/config/extrinsics.yaml`
- Create: `tests/test_go2_contract.py`

**Interfaces:**
- Consumes: `/lio/odometry`, `/lio/cloud_registered_body`, GO2 calibration, and `/cmd_vel_safe`.
- Produces: `/odom_nav`, `/cloud_registered_base`, static sensor TF, `odom -> base_footprint -> base_link`, and Unitree SDK2 motion calls.

- [ ] **Step 1: Write failing configuration and topic-contract tests**

```python
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]


def load_yaml(path):
    return yaml.safe_load((ROOT / path).read_text(encoding="utf-8"))


def test_go2_geometry_is_robot_specific():
    cfg = load_yaml("src/go2_system_bringup/config/go2_geometry.yaml")
    assert cfg["robot"]["name"] == "go2"
    assert cfg["robot"]["footprint"]["length"] > 0
    assert cfg["robot"]["footprint"]["width"] > 0


def test_command_chain_topics_are_preserved():
    text = "\n".join(p.read_text(errors="ignore") for p in (ROOT / "src").rglob("*") if p.is_file())
    assert "/cmd_vel_nav" in text
    assert "/cmd_vel_safe" in text
```

- [ ] **Step 2: Run tests and confirm missing new configuration fails**

```bash
python3 -m pytest -q tests/test_go2_contract.py
```

- [ ] **Step 3: Port existing GO2 implementations without Scout hardware values**

Use current GO2 calibration as the source of truth. The public interface must be exactly:

```yaml
topics:
  lio_odom: /lio/odometry
  lio_cloud_body: /lio/cloud_registered_body
  nav_odom: /odom_nav
  cloud_base: /cloud_registered_base
  planner_cmd: /cmd_vel_nav
  safe_cmd: /cmd_vel_safe
frames:
  map: map
  odom: odom
  base_footprint: base_footprint
  base_link: base_link
  lidar: lidar_link
```

Replace `/home/nvidia/unitree_sdk2_src/unitree_sdk2-main` in `go2_sdk_bridge/CMakeLists.txt` with a cache variable whose default is `${CMAKE_CURRENT_SOURCE_DIR}/../Unitree_SDK2`, and fail with a readable CMake error when the SDK is absent.

- [ ] **Step 4: Enforce initial motion limits consistently**

Use these initial values in watchdog and SDK bridge configuration:

```yaml
max_vx: 0.20
max_vy: 0.0
max_wz: 0.30
cmd_timeout_sec: 0.50
```

- [ ] **Step 5: Build and test only the GO2 adapter/control layer**

```bash
python3 -m pytest -q tests/test_go2_contract.py
catkin_make -j1 --pkg go2_tf_manager go2_pose_adapter go2_cloud_adapter go2_cmd_watchdog go2_sdk_bridge
```

- [ ] **Step 6: Commit**

```bash
git add src/go2_* tests/test_go2_contract.py
git commit -m "refactor: port GO2 hardware and frame adapters"
```

---

### Task 4: Implement Scout-Style Mapping and Map Finalization

**Files:**
- Modify: `src/FAST_LIO/config/mid360.yaml`
- Modify: `src/go2_pointcloud_mapper/**`
- Modify: `src/go2_map_tools/**`
- Create: `src/go2_system_bringup/launch/go2_mapping.launch`
- Create: `tests/test_mapping_contract.py`

**Interfaces:**
- Consumes: Livox point cloud/IMU and GO2 frame-adapted registered cloud.
- Produces: `maps/<map_name>/public_map.pcd`, `map_raw.pgm`, `map_raw.yaml`, and navigation map assets.

- [ ] **Step 1: Write failing mapping-contract tests**

```python
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_mapping_disables_fast_lio_native_pcd():
    text = (ROOT / "src/go2_system_bringup/launch/go2_mapping.launch").read_text()
    assert 'name="pcd_save/pcd_save_en" value="false"' in text
    assert "go2_pointcloud_mapper" in text


def test_mapping_has_no_absolute_project_path():
    for base in (ROOT / "src/go2_pointcloud_mapper", ROOT / "src/go2_map_tools"):
        for path in base.rglob("*"):
            if path.is_file():
                assert "/home/nvidia/go2_mid360_nav" not in path.read_text(errors="ignore")
```

- [ ] **Step 2: Verify tests fail before the new launch and path cleanup**

```bash
python3 -m pytest -q tests/test_mapping_contract.py
```

- [ ] **Step 3: Port Scout mapper behavior using GO2 topics and frames**

The launch interface must be:

```xml
<arg name="map_name" default="current_mapping"/>
<arg name="map_root" default="$(env HOME)/go2_mid360_nav/maps"/>
<arg name="map_dir" default="$(arg map_root)/$(arg map_name)"/>
```

The mapper subscribes to `/cloud_registered_base`, filters dynamic/outlier points as in Scout, and writes only when its explicit save/finalize action is requested.

- [ ] **Step 4: Make map tools parameter-driven**

Remove all compiled-in `/home/nvidia/go2_mid360_nav` defaults. `finalize_map.py <map_name>` resolves `map_root` from a ROS parameter or CLI option and atomically writes final files via temporary names followed by rename.

- [ ] **Step 5: Run static tests and build**

```bash
python3 -m pytest -q tests/test_mapping_contract.py
catkin_make -j1 --pkg fast_lio go2_pointcloud_mapper go2_map_tools go2_system_bringup
```

- [ ] **Step 6: Commit**

```bash
git add src/FAST_LIO src/go2_pointcloud_mapper src/go2_map_tools src/go2_system_bringup tests/test_mapping_contract.py
git commit -m "feat: add GO2 Scout-style mapping workflow"
```

---

### Task 5: Port NDT Global Localization with Unique TF Ownership

**Files:**
- Modify: `src/fast_lio_localization/**`
- Create: `src/go2_system_bringup/launch/go2_relocalization.launch`
- Create: `src/go2_system_bringup/launch/go2_localization.launch`
- Create: `tests/test_localization_contract.py`

**Interfaces:**
- Consumes: `/cloud_registered_base`, `/odom_nav`, static PCD, and initial pose.
- Produces: `/localization`, `/localization/ok`, and the sole `map -> odom` TF.

- [ ] **Step 1: Write failing localization-contract tests**

```python
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def all_active_text():
    return {p: p.read_text(errors="ignore") for p in (ROOT / "src").rglob("*") if p.is_file()}


def test_only_ndt_package_declares_map_to_odom_broadcast():
    hits = [p for p, text in all_active_text().items() if "map" in text and "odom" in text and "sendTransform" in text]
    assert hits
    assert all("fast_lio_localization" in str(p) for p in hits), hits


def test_localization_uses_go2_public_inputs():
    text = (ROOT / "src/go2_system_bringup/launch/go2_relocalization.launch").read_text()
    assert "/cloud_registered_base" in text
    assert "/odom_nav" in text
```

- [ ] **Step 2: Verify the tests fail before localization launch adaptation**

```bash
python3 -m pytest -q tests/test_localization_contract.py
```

- [ ] **Step 3: Adapt Scout NDT package to GO2 frames and health output**

Use these public parameters and defaults:

```yaml
map_frame: map
odom_frame: odom
base_frame: base_link
cloud_topic: /cloud_registered_base
odom_topic: /odom_nav
localization_topic: /localization
health_topic: /localization/ok
tf_postdate_sec: 0.50
```

Preserve Scout NDT-OMP implementation and expose convergence score, iteration count, translation jump, rotation jump, and last-success age to `go2_localization_guard`.

- [ ] **Step 4: Remove competing public `map -> odom` publishers from active launches**

Do not include old `go2_relocalization_bridge` or FAST_LIO localization transform-fusion nodes in the active `src/` launch graph.

- [ ] **Step 5: Build and run static tests**

```bash
python3 -m pytest -q tests/test_localization_contract.py
catkin_make -j1 --pkg fast_lio_localization go2_localization_guard go2_system_bringup
```

- [ ] **Step 6: Commit**

```bash
git add src/fast_lio_localization src/go2_localization_guard src/go2_system_bringup tests/test_localization_contract.py
git commit -m "feat: add NDT global localization for GO2"
```

---

### Task 6: Replace EGO-Planner with Non-Holonomic move_base and TEB

**Files:**
- Create: `src/go2_navigation/CMakeLists.txt`
- Create: `src/go2_navigation/package.xml`
- Create: `src/go2_navigation/config/costmap_common.yaml`
- Create: `src/go2_navigation/config/global_costmap.yaml`
- Create: `src/go2_navigation/config/local_costmap.yaml`
- Create: `src/go2_navigation/config/global_planner.yaml`
- Create: `src/go2_navigation/config/move_base_teb.yaml`
- Create: `src/go2_navigation/config/teb_local_planner.yaml`
- Create: `src/go2_navigation/launch/navigation_teb.launch`
- Create: `tests/test_navigation_contract.py`

**Interfaces:**
- Consumes: `/map_2d`, `map -> odom -> base_link`, `/odom_nav`, and obstacle point cloud.
- Produces: `/cmd_vel_nav`; never publishes directly to `/cmd_vel_safe`.

- [ ] **Step 1: Write failing navigation safety tests**

```python
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_teb_is_non_holonomic_and_bounded():
    cfg = yaml.safe_load((ROOT / "src/go2_navigation/config/teb_local_planner.yaml").read_text())
    teb = cfg["TebLocalPlannerROS"]
    assert teb["max_vel_y"] == 0.0
    assert teb["acc_lim_y"] == 0.0
    assert teb["max_vel_x"] <= 0.20
    assert teb["max_vel_theta"] <= 0.30
    assert teb["odom_topic"] == "/odom_nav"


def test_move_base_output_enters_watchdog_only():
    text = (ROOT / "src/go2_navigation/launch/navigation_teb.launch").read_text()
    assert 'from="cmd_vel" to="/cmd_vel_nav"' in text
    assert "/cmd_vel_safe" not in text
```

- [ ] **Step 2: Verify tests fail because the package does not exist**

```bash
python3 -m pytest -q tests/test_navigation_contract.py
```

- [ ] **Step 3: Port Scout navigation files and replace robot-specific values**

Start from Scout files but set:

```yaml
TebLocalPlannerROS:
  odom_topic: /odom_nav
  map_frame: map
  max_vel_x: 0.20
  max_vel_x_backwards: 0.0
  max_vel_y: 0.0
  max_vel_theta: 0.30
  acc_lim_x: 0.30
  acc_lim_y: 0.0
  acc_lim_theta: 0.50
  min_turning_radius: 0.0
```

Load GO2 footprint dimensions from `go2_geometry.yaml`; do not retain Scout wheelbase, footprint, velocity, or odometry topic values.

- [ ] **Step 4: Wire costmaps and launch output**

Use `map` as global frame, `odom` as local frame, `base_link` as robot base frame, and a remap from `cmd_vel` to `/cmd_vel_nav`. Include `go2_cmd_watchdog`; do not include the real SDK bridge in the navigation launch.

- [ ] **Step 5: Run tests, launch parsing, and package build**

```bash
python3 -m pytest -q tests/test_navigation_contract.py
roslaunch --nodes go2_navigation navigation_teb.launch map_name:=site01
catkin_make -j1 --pkg go2_navigation go2_cmd_watchdog
```

Expected: nodes include `move_base` and `go2_cmd_watchdog`; tests confirm zero lateral limits.

- [ ] **Step 6: Commit**

```bash
git add src/go2_navigation src/go2_cmd_watchdog tests/test_navigation_contract.py
git commit -m "feat: replace EGO navigation with move_base and TEB"
```

---

### Task 7: Canonical Bringup, Static Launch Validation, and Dry Run

**Files:**
- Modify: `src/go2_system_bringup/launch/go2_mapping.launch`
- Modify: `src/go2_system_bringup/launch/go2_localization.launch`
- Create: `src/go2_system_bringup/launch/go2_navigation.launch`
- Create: `scripts/validate_launch_contracts.py`
- Create: `scripts/check_cmd_vel.py`
- Modify: `README.md`

**Interfaces:**
- Consumes: all active packages from Tasks 2-6.
- Produces: three documented, canonical launch commands and a no-motion validation report.

- [ ] **Step 1: Write the failing launch-validator expectations**

```python
EXPECTED = {
    "go2_mapping.launch": {"laserMapping", "go2_pointcloud_mapper"},
    "go2_localization.launch": {"laserMapping", "fast_lio_localization", "go2_localization_guard"},
    "go2_navigation.launch": {"move_base", "go2_cmd_watchdog"},
}
```

The script runs `roslaunch --nodes`, compares exact required node subsets, and exits nonzero on missing nodes or any active EGO package reference.

- [ ] **Step 2: Run the validator and verify the incomplete launch graph fails**

```bash
python3 scripts/validate_launch_contracts.py
```

- [ ] **Step 3: Complete the canonical launch graph**

Requirements:

```text
mapping:      driver + FAST-LIO + GO2 TF/pose/cloud + pointcloud mapper
localization: driver + FAST-LIO + GO2 TF/pose/cloud + map server + NDT + guard
navigation:   localization precondition + move_base/TEB + watchdog
```

The real SDK bridge remains a separate launch and starts disabled.

- [ ] **Step 4: Add a no-motion command monitor**

`scripts/check_cmd_vel.py` subscribes to `/cmd_vel_nav` and fails if `abs(linear.y) > 1e-6`, any value is non-finite, or `vx/wz` exceeds configured limits. It never publishes commands.

- [ ] **Step 5: Run full clean build and no-motion checks**

```bash
cd /home/nvidia/go2_mid360_nav
rm -rf /home/nvidia/go2_mid360_nav/build /home/nvidia/go2_mid360_nav/devel
source /opt/ros/noetic/setup.bash
catkin_make -j1
source devel/setup.bash
python3 -m pytest -q tests
python3 scripts/validate_launch_contracts.py
git diff --check
```

Expected: clean build succeeds; all static tests pass; launch graph contains no EGO nodes; no real-motion launch is started.

- [ ] **Step 6: Commit**

```bash
git add README.md src/go2_system_bringup scripts tests
git commit -m "feat: add canonical GO2 mapping localization navigation bringup"
```

---

### Task 8: Repository Review and Non-Motion Acceptance Handoff

**Files:**
- Create: `docs/GO2_开发文档_V1.md`
- Create: `docs/GO2_使用手册_V1.md`
- Create: `docs/GO2_话题_启动文件_TF_检查表_V1.md`
- Create: `docs/acceptance/non_motion_acceptance.md`

**Interfaces:**
- Consumes: completed active workspace and test evidence.
- Produces: reviewed migration branch ready for sensor/mapping/localization testing, but not automatic real movement.

- [ ] **Step 1: Record exact acceptance evidence**

The acceptance document must contain command, timestamp, exit code, and concise output for:

```bash
git status --short --branch
git ls-files | grep -E '(^|/)(build|devel|maps|bags|datasets|logs)/' || true
git ls-files -z | xargs -0 -r du -b | sort -nr | head -n 20
python3 -m pytest -q tests
catkin_make -j1
python3 scripts/validate_launch_contracts.py
```

- [ ] **Step 2: Document operator safety sequence**

The usage manual must state this exact order:

```text
1. Start sensors and localization.
2. Set initial pose and verify map/cloud overlap.
3. Confirm /localization/ok is true and TF has one publisher per edge.
4. Start move_base/TEB with the real SDK bridge disabled.
5. Confirm /cmd_vel_nav limits and linear.y == 0.
6. Only in a later supervised test, start and manually enable the real SDK bridge.
```

- [ ] **Step 3: Run final repository checks**

```bash
git diff --check
git status --short
git log --oneline --decorate -10
```

Expected: only intended source/document changes are tracked; legacy untracked runtime directories remain untouched.

- [ ] **Step 4: Commit documentation**

```bash
git add docs README.md
git commit -m "docs: add GO2 Scout-style operating and acceptance guides"
```

- [ ] **Step 5: Stop before destructive cleanup or real movement**

Do not delete `catkin_ws`, `ego_ws`, `localization_ws`, `vendor`, or `third_party`. Do not enable the real SDK bridge. Report the branch, commit list, build/test evidence, and remaining supervised acceptance steps to the user.
