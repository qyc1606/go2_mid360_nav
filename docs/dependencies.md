# GO2 migration dependency baseline

Recorded on 2026-09-01 on the GO2 Jetson (Ubuntu 20.04, ROS Noetic, ARM64).

## Installed packages

```text
python3-pytest 4.6.9-1
ros-noetic-navigation 1.17.3-1focal.20250521.023218
ros-noetic-teb-local-planner 0.9.1-1focal.20250521.021738
```

## Source snapshots

```text
AADCL_UAV_UGV 0782a6dfa8ff407492cef63e4eb0e9f5ff0ebff9
GO2 pre-migration source snapshot 251c328d1f51a958a18cceb7055d52c46a815f44
```

The Scout Mini reference checkout is stored at
`/media/nvidia/系统/reference_projects/AADCL_UAV_UGV` and is treated as
read-only. FAST-LIO and Livox source provenance will be expanded in
`src/THIRD_PARTY.md` when the clean active source tree is imported.

## Host package-source repair

Before installing navigation dependencies, the stale RealSense APT entry was
updated from `librealsense.intel.com` to the current signed repository at
`librealsense.realsenseai.com`. The installed keyring contains signing key
`FB0B24895113F120`. The previous system source list is retained at
`/etc/apt/sources.list.pre_realsense_key_20260901`.
