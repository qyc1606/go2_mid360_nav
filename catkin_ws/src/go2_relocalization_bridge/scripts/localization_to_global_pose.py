#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import tf.transformations as tft
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


def pose_to_matrix(pose):
    """
    geometry_msgs/Pose -> 4x4 homogeneous transformation matrix
    """

    T = tft.translation_matrix([
        pose.position.x,
        pose.position.y,
        pose.position.z
    ])

    R = tft.quaternion_matrix([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    ])

    return np.dot(T, R)


def transform_to_matrix(transform):
    """
    geometry_msgs/Transform -> 4x4 homogeneous transformation matrix
    """

    T = tft.translation_matrix([
        transform.translation.x,
        transform.translation.y,
        transform.translation.z
    ])

    R = tft.quaternion_matrix([
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w
    ])

    return np.dot(T, R)


class LocalizationToGlobalPose:
    """
    Convert FAST-LIO localization pose:

        map -> body

    into GO2 navigation pose:

        map -> base_link

    and publish it as:

        /relocalization/global_pose
        geometry_msgs/PoseStamped

    The current MID-360 installation calibration provides:

        base_link -> lidar_link

    FAST-LIO uses an IMU/body frame whose rotation is aligned with
    the LiDAR frame in the current configuration because extrinsic_R = I.

    Therefore:

        R_base_body ~= R_base_lidar

    and:

        T_map_base =
            T_map_body * T_body_base

    where:

        T_body_base =
            inv(T_base_body)

    The node also drops duplicate input timestamps. This is required
    because transform_fusion may publish /localization faster than
    FAST-LIO updates /Odometry, while retaining the same measurement
    timestamp.
    """

    def __init__(self):
        rospy.init_node(
            "localization_to_global_pose",
            anonymous=False
        )

        # ============================================================
        # Parameters
        # ============================================================

        self.input_topic = rospy.get_param(
            "~input_topic",
            "/localization"
        )

        self.output_topic = rospy.get_param(
            "~output_topic",
            "/relocalization/global_pose"
        )

        self.map_frame = rospy.get_param(
            "~map_frame",
            "map"
        )

        self.base_frame = rospy.get_param(
            "~base_frame",
            "base_link"
        )

        self.body_frame = rospy.get_param(
            "~body_frame",
            "body"
        )

        self.lidar_frame = rospy.get_param(
            "~lidar_frame",
            "lidar_link"
        )

        # Whether to reject repeated measurement timestamps.
        self.drop_duplicate_stamp = rospy.get_param(
            "~drop_duplicate_stamp",
            True
        )

        # ============================================================
        # TF
        # ============================================================

        self.tf_buffer = tf2_ros.Buffer(
            cache_time=rospy.Duration(10.0)
        )

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer
        )

        # ============================================================
        # State
        # ============================================================

        self.T_body_base = None

        self.last_input_stamp = None

        self.received_count = 0
        self.published_count = 0
        self.duplicate_count = 0

        # ============================================================
        # ROS publisher / subscriber
        # ============================================================

        self.pub = rospy.Publisher(
            self.output_topic,
            PoseStamped,
            queue_size=10
        )

        self.sub = rospy.Subscriber(
            self.input_topic,
            Odometry,
            self.localization_callback,
            queue_size=20
        )

        # ============================================================
        # Startup information
        # ============================================================

        rospy.loginfo(
            "================================================"
        )

        rospy.loginfo(
            "localization_to_global_pose started"
        )

        rospy.loginfo(
            "Input topic : %s",
            self.input_topic
        )

        rospy.loginfo(
            "Output topic: %s",
            self.output_topic
        )

        rospy.loginfo(
            "Input pose semantics : %s -> %s",
            self.map_frame,
            self.body_frame
        )

        rospy.loginfo(
            "Output pose semantics: %s -> %s",
            self.map_frame,
            self.base_frame
        )

        rospy.loginfo(
            "Calibration TF used: %s -> %s",
            self.base_frame,
            self.lidar_frame
        )

        rospy.loginfo(
            "Drop duplicate timestamp: %s",
            str(self.drop_duplicate_stamp)
        )

        rospy.loginfo(
            "================================================"
        )

    # ================================================================
    # Load body -> base transform
    # ================================================================

    def update_body_to_base(self):
        """
        Read calibrated TF:

            base_link -> lidar_link

        Current FAST-LIO configuration uses:

            extrinsic_R = I

        so the FAST-LIO body axes and LiDAR axes are aligned in rotation.

        For the present navigation-frame correction:

            base_link -> body
            ~=
            base_link -> lidar_link

        hence:

            body -> base_link
            =
            inverse(base_link -> body)

        The few-centimeter FAST-LIO internal IMU-LiDAR translation is
        intentionally not merged here yet; this node first guarantees
        the correct navigation-frame orientation and consistent TF tree.
        """

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.lidar_frame,
                rospy.Time(0),
                rospy.Duration(1.0)
            )

        except Exception as exc:
            rospy.logwarn_throttle(
                2.0,
                "Waiting for TF %s -> %s: %s",
                self.base_frame,
                self.lidar_frame,
                str(exc)
            )
            return False

        T_base_lidar = transform_to_matrix(
            tf_msg.transform
        )

        # ------------------------------------------------------------
        # Current FAST-LIO body rotation is aligned with lidar_link.
        #
        # Therefore:
        #
        #     T_base_body ~= T_base_lidar
        #
        # and:
        #
        #     T_body_base = inv(T_base_body)
        # ------------------------------------------------------------

        T_base_body = T_base_lidar

        self.T_body_base = np.linalg.inv(
            T_base_body
        )

        # Print the calibration once.
        q = tft.quaternion_from_matrix(
            self.T_body_base
        )

        roll, pitch, yaw = tft.euler_from_quaternion(
            q
        )

        rospy.loginfo(
            "Loaded body -> base_link correction."
        )

        rospy.loginfo(
            "body -> base_link translation = "
            "[%.4f, %.4f, %.4f] m",
            self.T_body_base[0, 3],
            self.T_body_base[1, 3],
            self.T_body_base[2, 3]
        )

        rospy.loginfo(
            "body -> base_link RPY = "
            "[%.3f, %.3f, %.3f] deg",
            np.degrees(roll),
            np.degrees(pitch),
            np.degrees(yaw)
        )

        return True

    # ================================================================
    # Localization callback
    # ================================================================

    def localization_callback(self, msg):
        self.received_count += 1

        stamp = msg.header.stamp

        # ------------------------------------------------------------
        # Drop repeated measurement timestamps.
        #
        # transform_fusion may publish at ~40-50 Hz while FAST-LIO
        # odometry updates at ~10 Hz. Several /localization messages
        # may therefore carry the exact same timestamp.
        #
        # Forwarding all of them causes:
        #
        # TF_REPEATED_DATA ignoring data with redundant timestamp
        #
        # in go2_relocalization_bridge.
        # ------------------------------------------------------------

        if self.drop_duplicate_stamp:

            if (
                self.last_input_stamp is not None
                and stamp == self.last_input_stamp
            ):
                self.duplicate_count += 1

                rospy.logdebug_throttle(
                    5.0,
                    "Dropped repeated localization timestamp. "
                    "received=%d published=%d duplicates=%d",
                    self.received_count,
                    self.published_count,
                    self.duplicate_count
                )

                return

        self.last_input_stamp = stamp

        # ------------------------------------------------------------
        # Load calibration transform once.
        # ------------------------------------------------------------

        if self.T_body_base is None:

            if not self.update_body_to_base():
                return

        # ------------------------------------------------------------
        # Input:
        #
        #     /localization
        #
        # represents:
        #
        #     map -> body
        # ------------------------------------------------------------

        T_map_body = pose_to_matrix(
            msg.pose.pose
        )

        # ------------------------------------------------------------
        # Convert:
        #
        #     map -> body
        #
        # to:
        #
        #     map -> base_link
        #
        # using:
        #
        #     T_map_base =
        #         T_map_body * T_body_base
        # ------------------------------------------------------------

        T_map_base = np.dot(
            T_map_body,
            self.T_body_base
        )

        # ------------------------------------------------------------
        # Convert matrix to PoseStamped
        # ------------------------------------------------------------

        q = tft.quaternion_from_matrix(
            T_map_base
        )

        out = PoseStamped()

        # Preserve original measurement timestamp.
        #
        # Do NOT replace this with rospy.Time.now().
        #
        # A navigation pose should retain the timestamp of the
        # measurement from which it was calculated.
        out.header.stamp = stamp

        # Fallback only in the abnormal case of zero timestamp.
        if out.header.stamp == rospy.Time():
            out.header.stamp = rospy.Time.now()

        out.header.frame_id = self.map_frame

        out.pose.position.x = float(
            T_map_base[0, 3]
        )

        out.pose.position.y = float(
            T_map_base[1, 3]
        )

        out.pose.position.z = float(
            T_map_base[2, 3]
        )

        out.pose.orientation.x = float(
            q[0]
        )

        out.pose.orientation.y = float(
            q[1]
        )

        out.pose.orientation.z = float(
            q[2]
        )

        out.pose.orientation.w = float(
            q[3]
        )

        self.pub.publish(
            out
        )

        self.published_count += 1

        # ------------------------------------------------------------
        # Periodic status
        # ------------------------------------------------------------

        rospy.loginfo_throttle(
            5.0,
            "global_pose: "
            "received=%d published=%d duplicates=%d "
            "pos=[%.3f %.3f %.3f]",
            self.received_count,
            self.published_count,
            self.duplicate_count,
            out.pose.position.x,
            out.pose.position.y,
            out.pose.position.z
        )


def main():

    try:
        LocalizationToGlobalPose()

        rospy.loginfo(
            "localization_to_global_pose ready."
        )

        rospy.spin()

    except rospy.ROSInterruptException:
        pass

    except Exception as exc:

        rospy.logfatal(
            "localization_to_global_pose fatal error: %s",
            str(exc)
        )

        raise


if __name__ == "__main__":
    main()
