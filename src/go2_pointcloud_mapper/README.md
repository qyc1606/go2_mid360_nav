# go2_pointcloud_mapper

This node subscribes to FAST-LIO's registered world-frame scan and odometry,
filters outliers, and maintains a lightweight 3D Bayesian occupancy grid. Scan
endpoints increase occupancy confidence; sampled free-space rays decrease it.
Fine map points are published only while their coarse occupancy voxel is both
probable and stable, so departed people and robots can be removed. It does not
publish TF and never feeds points back to FAST-LIO.

The normal mapping launch starts this node automatically. It writes no map on a
timer or during shutdown. Save the finished map explicitly with:

```bash
rosservice call /go2_pointcloud_mapper/save_map
```

Reset all candidate and confirmed voxels:

```bash
rosservice call /go2_pointcloud_mapper/reset_map
```

Start a named mapping session with:

```bash
roslaunch go2_system_bringup go2_mapping.launch map_name:=go2_map_01
```

The default mapping launch writes to:

```text
~/go2_mid360_nav/maps/current_mapping/public_map.pcd
```

The named launch writes `public_map.pcd` below its named map directory. Run
`scripts/finalize_map.py go2_map_01` to generate localization and navigation
assets.

The default dynamic filter requires at least eight hit scans spanning two
seconds. Free-space clearing traces one quarter of the filtered points out to
20 m to limit Jetson CPU use. A departed object is removed only after the lidar
observes free space through its former position; revisit occluded areas before
finalizing a map.
