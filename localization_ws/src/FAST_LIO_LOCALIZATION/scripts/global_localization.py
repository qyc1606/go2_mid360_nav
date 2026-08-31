#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
FAST_LIO_LOCALIZATION - GO2 / Livox MID-360 global localization

Inputs
------
/map
    sensor_msgs/PointCloud2
    Frozen global PCD map, frame_id = "map"

/cloud_registered
    sensor_msgs/PointCloud2
    FAST-LIO registered current cloud, frame_id = "camera_init"

/Odometry
    nav_msgs/Odometry
    FAST-LIO local odometry:
        header.frame_id = "camera_init"
        child_frame_id  = "body"

/initialpose
    geometry_msgs/PoseWithCovarianceStamped
    Initial robot pose supplied from RViz "2D Pose Estimate",
    expressed in the map frame.

Output
------
/map_to_odom
    nav_msgs/Odometry
    Pose of FAST-LIO local world frame "camera_init"
    expressed in global frame "map".

Transform convention
--------------------
T_map_body = T_map_camera_init * T_camera_init_body

Therefore:

T_map_camera_init =
    T_map_body * inv(T_camera_init_body)

The ICP refinement directly estimates the transform from
camera_init coordinates to map coordinates because
/cloud_registered is expressed in camera_init.
"""

import threading
import time

import numpy as np
import open3d as o3d
import rospy
import tf.transformations as tft

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2


class GlobalLocalizationNode:
    def __init__(self):
        rospy.init_node("global_localization", anonymous=False)

        # ------------------------------------------------------------
        # Topics
        # ------------------------------------------------------------
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.scan_topic = rospy.get_param(
            "~scan_topic",
            "/cloud_registered"
        )
        self.odom_topic = rospy.get_param(
            "~odom_topic",
            "/Odometry"
        )
        self.initialpose_topic = rospy.get_param(
            "~initialpose_topic",
            "/initialpose"
        )
        self.output_topic = rospy.get_param(
            "~output_topic",
            "/map_to_odom"
        )

        # ------------------------------------------------------------
        # Frames
        # ------------------------------------------------------------
        self.map_frame = rospy.get_param(
            "~map_frame",
            "map"
        )
        self.odom_frame = rospy.get_param(
            "~odom_frame",
            "camera_init"
        )
        self.body_frame = rospy.get_param(
            "~body_frame",
            "body"
        )

        # ------------------------------------------------------------
        # Point cloud preprocessing
        # ------------------------------------------------------------
        self.map_voxel_size = rospy.get_param(
            "~map_voxel_size",
            0.20
        )
        self.scan_voxel_size = rospy.get_param(
            "~scan_voxel_size",
            0.15
        )

        self.max_scan_range = rospy.get_param(
            "~max_scan_range",
            50.0
        )

        # MID-360 is treated as a 360-degree LiDAR.
        self.fov_rad = rospy.get_param(
            "~fov_rad",
            2.0 * np.pi
        )

        # ------------------------------------------------------------
        # ICP
        #
        # Two-stage ICP:
        #   coarse -> fine
        # ------------------------------------------------------------
        self.coarse_max_corr = rospy.get_param(
            "~coarse_max_correspondence",
            2.0
        )
        self.fine_max_corr = rospy.get_param(
            "~fine_max_correspondence",
            0.60
        )

        self.coarse_iterations = rospy.get_param(
            "~coarse_iterations",
            40
        )
        self.fine_iterations = rospy.get_param(
            "~fine_iterations",
            50
        )

        # Open3D fitness:
        # fraction of source points that obtain valid correspondences.
        #
        # Do not use the old hard-coded 0.95 initially.
        # Real MID-360 scan-to-map registration usually should be
        # evaluated together with RMSE.
        self.min_fitness = rospy.get_param(
            "~min_fitness",
            0.45
        )

        self.max_rmse = rospy.get_param(
            "~max_rmse",
            0.50
        )

        # ------------------------------------------------------------
        # Continuous relocalization
        # ------------------------------------------------------------
        self.enable_periodic_refine = rospy.get_param(
            "~enable_periodic_refine",
            True
        )

        self.refine_period = rospy.get_param(
            "~refine_period",
            2.0
        )

        # Prevent one bad ICP update from causing a large pose jump.
        self.max_update_translation = rospy.get_param(
            "~max_update_translation",
            1.0
        )

        self.max_update_yaw_deg = rospy.get_param(
            "~max_update_yaw_deg",
            20.0
        )

        # ------------------------------------------------------------
        # State
        # ------------------------------------------------------------
        self.lock = threading.Lock()

        self.global_map_np = None
        self.global_map_o3d = None

        self.latest_scan_np = None
        self.latest_scan_stamp = None

        self.latest_odom = None

        self.T_map_odom = np.eye(4, dtype=np.float64)

        self.initialized = False
        self.initializing = False

        self.last_refine_time = 0.0

        # ------------------------------------------------------------
        # ROS
        # ------------------------------------------------------------
        self.pub_map_to_odom = rospy.Publisher(
            self.output_topic,
            Odometry,
            queue_size=10
        )

        self.sub_map = rospy.Subscriber(
            self.map_topic,
            PointCloud2,
            self.map_callback,
            queue_size=1
        )

        self.sub_scan = rospy.Subscriber(
            self.scan_topic,
            PointCloud2,
            self.scan_callback,
            queue_size=1,
            buff_size=2 ** 26
        )

        self.sub_odom = rospy.Subscriber(
            self.odom_topic,
            Odometry,
            self.odom_callback,
            queue_size=20
        )

        self.sub_initialpose = rospy.Subscriber(
            self.initialpose_topic,
            PoseWithCovarianceStamped,
            self.initialpose_callback,
            queue_size=1
        )

        self.timer = rospy.Timer(
            rospy.Duration(0.05),
            self.publish_timer
        )

        self.refine_timer = rospy.Timer(
            rospy.Duration(0.5),
            self.refine_timer_callback
        )

        rospy.loginfo(
            "Global localization node initialized "
            "(Python3 / Open3D %s)",
            o3d.__version__
        )

        rospy.loginfo(
            "Topics: map=%s scan=%s odom=%s initialpose=%s output=%s",
            self.map_topic,
            self.scan_topic,
            self.odom_topic,
            self.initialpose_topic,
            self.output_topic
        )

        rospy.loginfo(
            "Frames: %s -> %s -> %s",
            self.map_frame,
            self.odom_frame,
            self.body_frame
        )

        rospy.loginfo(
            "Waiting for global map / scan / odometry / initial pose..."
        )

    # ================================================================
    # Utility: PointCloud2 -> numpy
    # ================================================================
    @staticmethod
    def pointcloud2_to_xyz(msg):
        """
        Convert sensor_msgs/PointCloud2 to Nx3 float64 numpy array.

        ros_numpy is deliberately NOT used because ROS Noetic ros_numpy
        commonly uses deprecated NumPy aliases such as np.float,
        which fail with NumPy >= 1.24.
        """

        points = []

        try:
            for p in point_cloud2.read_points(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=True
            ):
                x = float(p[0])
                y = float(p[1])
                z = float(p[2])

                if (
                    np.isfinite(x)
                    and np.isfinite(y)
                    and np.isfinite(z)
                ):
                    points.append((x, y, z))

        except Exception as exc:
            rospy.logerr_throttle(
                2.0,
                "PointCloud2 conversion failed: %s",
                str(exc)
            )
            return None

        if not points:
            return None

        return np.asarray(
            points,
            dtype=np.float64
        )

    # ================================================================
    # Utility: numpy -> Open3D
    # ================================================================
    @staticmethod
    def xyz_to_o3d(points):
        cloud = o3d.geometry.PointCloud()

        if points is not None and len(points) > 0:
            cloud.points = o3d.utility.Vector3dVector(
                np.asarray(points, dtype=np.float64)
            )

        return cloud

    # ================================================================
    # Utility: geometry pose -> homogeneous transform
    # ================================================================
    @staticmethod
    def pose_to_matrix(pose):
        translation = tft.translation_matrix([
            pose.position.x,
            pose.position.y,
            pose.position.z
        ])

        rotation = tft.quaternion_matrix([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])

        return np.dot(
            translation,
            rotation
        )

    # ================================================================
    # Utility: homogeneous transform -> pose
    # ================================================================
    @staticmethod
    def matrix_to_pose_fields(T):
        q = tft.quaternion_from_matrix(T)

        position = T[:3, 3]

        return position, q

    # ================================================================
    # Map callback
    # ================================================================
    def map_callback(self, msg):
        with self.lock:
            # The frozen PCD map is static. Only process it once.
            if self.global_map_o3d is not None:
                return

        rospy.loginfo(
            "Receiving global map on %s ...",
            self.map_topic
        )

        xyz = self.pointcloud2_to_xyz(msg)

        if xyz is None or len(xyz) < 100:
            rospy.logwarn(
                "Global map contains too few valid points."
            )
            return

        cloud = self.xyz_to_o3d(xyz)

        if self.map_voxel_size > 0.0:
            cloud = cloud.voxel_down_sample(
                self.map_voxel_size
            )

        map_np = np.asarray(
            cloud.points,
            dtype=np.float64
        )

        if len(map_np) < 100:
            rospy.logwarn(
                "Global map contains too few points after downsampling."
            )
            return

        with self.lock:
            self.global_map_np = map_np
            self.global_map_o3d = cloud

        rospy.loginfo(
            "Global map loaded: raw=%d, downsampled=%d",
            len(xyz),
            len(map_np)
        )

    # ================================================================
    # Scan callback
    # ================================================================
    def scan_callback(self, msg):
        xyz = self.pointcloud2_to_xyz(msg)

        if xyz is None or len(xyz) < 50:
            return

        # Range filtering in FAST-LIO local-world coordinates.
        # We only remove pathological / extremely distant points here.
        dist = np.linalg.norm(
            xyz,
            axis=1
        )

        mask = np.logical_and(
            np.isfinite(dist),
            dist < self.max_scan_range
        )

        xyz = xyz[mask]

        if len(xyz) < 50:
            return

        with self.lock:
            self.latest_scan_np = xyz
            self.latest_scan_stamp = msg.header.stamp

    # ================================================================
    # Odometry callback
    # ================================================================
    def odom_callback(self, msg):
        with self.lock:
            self.latest_odom = msg

    # ================================================================
    # Build initial map->odom transform from RViz initial pose
    # ================================================================
    def initialpose_callback(self, msg):
        rospy.loginfo(
            "Received /initialpose. Starting global localization..."
        )

        with self.lock:
            if self.initializing:
                rospy.logwarn(
                    "Initialization already running."
                )
                return

            if self.global_map_o3d is None:
                rospy.logwarn(
                    "Cannot initialize: global map not received."
                )
                return

            if self.latest_scan_np is None:
                rospy.logwarn(
                    "Cannot initialize: current scan not received."
                )
                return

            if self.latest_odom is None:
                rospy.logwarn(
                    "Cannot initialize: FAST-LIO odometry not received."
                )
                return

            self.initializing = True

            scan_np = self.latest_scan_np.copy()
            odom_msg = self.latest_odom
            map_cloud = self.global_map_o3d

        try:
            # --------------------------------------------------------
            # RViz /initialpose represents the BODY pose in MAP.
            # --------------------------------------------------------
            T_map_body_guess = self.pose_to_matrix(
                msg.pose.pose
            )

            # FAST-LIO Odometry represents BODY pose in CAMERA_INIT.
            T_odom_body = self.pose_to_matrix(
                odom_msg.pose.pose
            )

            # map -> odom initial guess:
            #
            # T_map_body =
            # T_map_odom * T_odom_body
            #
            # therefore:
            #
            # T_map_odom =
            # T_map_body * inv(T_odom_body)
            T_initial = np.dot(
                T_map_body_guess,
                np.linalg.inv(T_odom_body)
            )

            success, T_refined, fitness, rmse = self.run_icp(
                scan_np,
                map_cloud,
                T_initial,
                verbose=True
            )

            if not success:
                rospy.logwarn(
                    "Global initialization FAILED: "
                    "fitness=%.4f rmse=%.4f",
                    fitness,
                    rmse
                )

                rospy.logwarn(
                    "Please provide a more accurate "
                    "RViz 2D Pose Estimate."
                )

                return

            with self.lock:
                self.T_map_odom = T_refined
                self.initialized = True
                self.last_refine_time = time.time()

            rospy.loginfo(
                "================================================"
            )
            rospy.loginfo(
                "GLOBAL LOCALIZATION INITIALIZED SUCCESSFULLY"
            )
            rospy.loginfo(
                "fitness = %.4f",
                fitness
            )
            rospy.loginfo(
                "inlier_rmse = %.4f m",
                rmse
            )
            rospy.loginfo(
                "================================================"
            )

        except Exception as exc:
            rospy.logerr(
                "Global localization initialization exception: %s",
                str(exc)
            )

        finally:
            with self.lock:
                self.initializing = False

    # ================================================================
    # Crop global map around current scan estimate
    # ================================================================
    def crop_map_for_scan(
        self,
        scan_np,
        map_cloud,
        T_guess
    ):
        """
        Crop the global map around the current transformed scan.

        This avoids performing ICP against the entire frozen map and is
        considerably faster on Jetson/Orin.
        """

        transformed_scan = (
            np.dot(
                T_guess[:3, :3],
                scan_np.T
            ).T
            + T_guess[:3, 3]
        )

        if len(transformed_scan) == 0:
            return map_cloud

        scan_min = np.min(
            transformed_scan,
            axis=0
        )

        scan_max = np.max(
            transformed_scan,
            axis=0
        )

        # Add spatial margin around current scan.
        margin = np.array(
            [8.0, 8.0, 4.0],
            dtype=np.float64
        )

        min_bound = scan_min - margin
        max_bound = scan_max + margin

        bbox = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=min_bound,
            max_bound=max_bound
        )

        cropped = map_cloud.crop(
            bbox
        )

        if len(cropped.points) < 500:
            rospy.logwarn_throttle(
                2.0,
                "Local map crop too small; using full global map."
            )
            return map_cloud

        return cropped

    # ================================================================
    # ICP
    # ================================================================
    def run_icp(
        self,
        scan_np,
        global_map,
        initial_transform,
        verbose=False
    ):
        if scan_np is None or len(scan_np) < 50:
            return False, initial_transform, 0.0, float("inf")

        source = self.xyz_to_o3d(
            scan_np
        )

        if self.scan_voxel_size > 0.0:
            source = source.voxel_down_sample(
                self.scan_voxel_size
            )

        if len(source.points) < 50:
            return False, initial_transform, 0.0, float("inf")

        target = self.crop_map_for_scan(
            np.asarray(source.points),
            global_map,
            initial_transform
        )

        # ------------------------------------------------------------
        # Coarse ICP
        # ------------------------------------------------------------
        coarse = o3d.pipelines.registration.registration_icp(
            source,
            target,
            self.coarse_max_corr,
            initial_transform,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=self.coarse_iterations
            )
        )

        # ------------------------------------------------------------
        # Fine ICP
        # ------------------------------------------------------------
        fine = o3d.pipelines.registration.registration_icp(
            source,
            target,
            self.fine_max_corr,
            coarse.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=self.fine_iterations
            )
        )

        fitness = float(
            fine.fitness
        )

        rmse = float(
            fine.inlier_rmse
        )

        if verbose:
            rospy.loginfo(
                "ICP coarse fitness=%.4f rmse=%.4f",
                coarse.fitness,
                coarse.inlier_rmse
            )

            rospy.loginfo(
                "ICP fine   fitness=%.4f rmse=%.4f",
                fitness,
                rmse
            )

        success = (
            fitness >= self.min_fitness
            and rmse <= self.max_rmse
        )

        return (
            success,
            np.asarray(
                fine.transformation,
                dtype=np.float64
            ),
            fitness,
            rmse
        )

    # ================================================================
    # Periodic ICP refinement
    # ================================================================
    def refine_timer_callback(self, _event):
        if not self.enable_periodic_refine:
            return

        with self.lock:
            if not self.initialized:
                return

            if self.initializing:
                return

            now = time.time()

            if (
                now - self.last_refine_time
                < self.refine_period
            ):
                return

            if self.latest_scan_np is None:
                return

            if self.global_map_o3d is None:
                return

            scan_np = self.latest_scan_np.copy()
            map_cloud = self.global_map_o3d
            T_previous = self.T_map_odom.copy()

            self.last_refine_time = now

        try:
            success, T_new, fitness, rmse = self.run_icp(
                scan_np,
                map_cloud,
                T_previous,
                verbose=False
            )

            if not success:
                rospy.logwarn_throttle(
                    5.0,
                    "Periodic ICP rejected: "
                    "fitness=%.3f rmse=%.3f",
                    fitness,
                    rmse
                )
                return

            if not self.update_is_reasonable(
                T_previous,
                T_new
            ):
                rospy.logwarn_throttle(
                    5.0,
                    "Periodic ICP rejected due to excessive pose jump."
                )
                return

            with self.lock:
                self.T_map_odom = T_new

            rospy.loginfo_throttle(
                5.0,
                "ICP refine OK: fitness=%.3f rmse=%.3f",
                fitness,
                rmse
            )

        except Exception as exc:
            rospy.logerr_throttle(
                2.0,
                "Periodic ICP exception: %s",
                str(exc)
            )

    # ================================================================
    # Reject unreasonable ICP corrections
    # ================================================================
    def update_is_reasonable(
        self,
        T_old,
        T_new
    ):
        delta = np.dot(
            np.linalg.inv(T_old),
            T_new
        )

        translation_jump = np.linalg.norm(
            delta[:3, 3]
        )

        _, _, yaw = tft.euler_from_matrix(
            delta
        )

        yaw_jump_deg = abs(
            np.degrees(yaw)
        )

        if translation_jump > self.max_update_translation:
            rospy.logwarn(
                "ICP translation jump %.3f m > %.3f m",
                translation_jump,
                self.max_update_translation
            )
            return False

        if yaw_jump_deg > self.max_update_yaw_deg:
            rospy.logwarn(
                "ICP yaw jump %.2f deg > %.2f deg",
                yaw_jump_deg,
                self.max_update_yaw_deg
            )
            return False

        return True

    # ================================================================
    # Publish /map_to_odom
    # ================================================================
    def publish_timer(self, _event):
        with self.lock:
            if not self.initialized:
                return

            T = self.T_map_odom.copy()

        position, quaternion = self.matrix_to_pose_fields(
            T
        )

        msg = Odometry()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.odom_frame

        msg.pose.pose.position.x = float(
            position[0]
        )
        msg.pose.pose.position.y = float(
            position[1]
        )
        msg.pose.pose.position.z = float(
            position[2]
        )

        msg.pose.pose.orientation.x = float(
            quaternion[0]
        )
        msg.pose.pose.orientation.y = float(
            quaternion[1]
        )
        msg.pose.pose.orientation.z = float(
            quaternion[2]
        )
        msg.pose.pose.orientation.w = float(
            quaternion[3]
        )

        self.pub_map_to_odom.publish(
            msg
        )


def main():
    try:
        GlobalLocalizationNode()

        rospy.loginfo(
            "Global localization ready."
        )

        rospy.spin()

    except rospy.ROSInterruptException:
        pass

    except Exception as exc:
        rospy.logfatal(
            "global_localization fatal error: %s",
            str(exc)
        )
        raise


if __name__ == "__main__":
    main()
