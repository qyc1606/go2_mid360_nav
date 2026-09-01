# Third-party source snapshots

The active `src/` tree vendors source snapshots so the Jetson build does not
depend on nested Git repositories. Do not edit third-party code without
recording the reason and affected files here.

| Component | Upstream | Snapshot used here | Local status |
|---|---|---|---|
| FAST-LIO | `https://github.com/hku-mars/FAST_LIO.git` | GO2 repository snapshot `251c328d1f51a958a18cceb7055d52c46a815f44` | GO2-specific `mid360.yaml`, `laserMapping.cpp`, and `preprocess.h` differ from the Scout copy. |
| Livox ROS Driver 2 | `https://github.com/Livox-SDK/livox_ros_driver2.git` | `4a1def929e5b59c7a8122d19fce6efba581ce9f7` | Existing GO2 Mid-360 configuration is retained. |
| Livox SDK2 | `https://github.com/Livox-SDK/Livox-SDK2.git` | AADCL snapshot `0782a6dfa8ff407492cef63e4eb0e9f5ff0ebff9` | Copied from the read-only Scout Mini reference tree. |
| FAST-LIO Localization / NDT-OMP | source carried by AADCL Scout Mini | AADCL snapshot `0782a6dfa8ff407492cef63e4eb0e9f5ff0ebff9` | To be adapted only at ROS topics, frames, health output, and launch parameters. |
| Unitree SDK2 | `https://github.com/unitreerobotics/unitree_sdk2.git` | `21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b` | Used only through `go2_sdk_bridge`; nested Git metadata and build products are excluded. |

Each component retains its upstream license files in its copied directory.
The AADCL reference repository remains at
`/media/nvidia/系统/reference_projects/AADCL_UAV_UGV` and must stay read-only.
