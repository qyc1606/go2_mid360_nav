#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include <mutex>
#include <string>


class RelocalizationBridge
{
public:
  RelocalizationBridge()
      : nh_(),
        pnh_("~"),
        have_odom_(false),
        have_global_pose_(false)
  {
    // ============================================================
    // Parameters
    // ============================================================

    pnh_.param<std::string>(
        "global_pose_topic",
        global_pose_topic_,
        "/relocalization/global_pose");

    pnh_.param<std::string>(
        "odom_topic",
        odom_topic_,
        "/odom_nav");

    pnh_.param<std::string>(
        "map_frame",
        map_frame_,
        "map");

    pnh_.param<std::string>(
        "odom_frame",
        odom_frame_,
        "odom");

    pnh_.param<std::string>(
        "base_frame",
        base_frame_,
        "base_link");

    pnh_.param<bool>(
        "publish_tf",
        publish_tf_,
        true);

    pnh_.param<double>(
        "max_odom_age_sec",
        max_odom_age_sec_,
        0.5);


    // ============================================================
    // Subscribers
    // ============================================================

    odom_sub_ =
        nh_.subscribe(
            odom_topic_,
            20,
            &RelocalizationBridge::odomCallback,
            this);


    pose_sub_ =
        nh_.subscribe(
            global_pose_topic_,
            20,
            &RelocalizationBridge::globalPoseCallback,
            this);


    // ============================================================
    // Startup information
    // ============================================================

    ROS_INFO(
        "================================================");

    ROS_INFO(
        "go2_relocalization_bridge V2 started");

    ROS_INFO(
        "global_pose_topic = %s",
        global_pose_topic_.c_str());

    ROS_INFO(
        "odom_topic        = %s",
        odom_topic_.c_str());

    ROS_INFO(
        "map_frame         = %s",
        map_frame_.c_str());

    ROS_INFO(
        "odom_frame        = %s",
        odom_frame_.c_str());

    ROS_INFO(
        "base_frame        = %s",
        base_frame_.c_str());

    ROS_INFO(
        "max_odom_age_sec  = %.3f",
        max_odom_age_sec_);

    ROS_INFO(
        "================================================");
  }


private:

  // ==============================================================
  // /odom_nav callback
  //
  // Semantics:
  //
  //   odom -> base_link
  //
  // ==============================================================
  void odomCallback(
      const nav_msgs::Odometry::ConstPtr& msg)
  {
    if (!msg->header.frame_id.empty() &&
        msg->header.frame_id != odom_frame_)
    {
      ROS_WARN_THROTTLE(
          2.0,
          "/odom_nav frame_id=[%s], expected [%s]",
          msg->header.frame_id.c_str(),
          odom_frame_.c_str());
    }


    std::lock_guard<std::mutex> lock(
        mutex_);


    latest_odom_ =
        *msg;


    have_odom_ =
        true;


    ROS_INFO_THROTTLE(
        5.0,
        "Received /odom_nav: "
        "pos=[%.3f %.3f %.3f]",
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);
  }


  // ==============================================================
  // /relocalization/global_pose callback
  //
  // Semantics:
  //
  //   map -> base_link
  //
  // Compute:
  //
  //   T_map_odom
  //       =
  //   T_map_base
  //       *
  //   inverse(T_odom_base)
  //
  // ==============================================================
  void globalPoseCallback(
      const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    if (msg->header.frame_id != map_frame_)
    {
      ROS_WARN_THROTTLE(
          1.0,
          "Global pose frame=[%s], expected=[%s]",
          msg->header.frame_id.c_str(),
          map_frame_.c_str());

      return;
    }


    nav_msgs::Odometry odom;


    {
      std::lock_guard<std::mutex> lock(
          mutex_);


      if (!have_odom_)
      {
        ROS_WARN_THROTTLE(
            1.0,
            "Waiting for %s",
            odom_topic_.c_str());

        return;
      }


      odom =
          latest_odom_;
    }


    // ============================================================
    // Check odometry freshness
    // ============================================================

    if (!odom.header.stamp.isZero())
    {
      const double odom_age =
          std::fabs(
              (ros::Time::now() -
               odom.header.stamp).toSec());


      if (odom_age >
          max_odom_age_sec_)
      {
        ROS_WARN_THROTTLE(
            1.0,
            "Latest odom is too old: "
            "age=%.3f s > %.3f s",
            odom_age,
            max_odom_age_sec_);

        return;
      }
    }


    // ============================================================
    // Convert:
    //
    // global_pose:
    //
    //   map -> base_link
    //
    // odom_nav:
    //
    //   odom -> base_link
    //
    // ============================================================

    tf2::Transform T_map_base;

    tf2::Transform T_odom_base;


    tf2::fromMsg(
        msg->pose,
        T_map_base);


    tf2::fromMsg(
        odom.pose.pose,
        T_odom_base);


    // ============================================================
    // Compute:
    //
    // T_map_odom =
    //
    // T_map_base *
    // inverse(T_odom_base)
    //
    // ============================================================

    const tf2::Transform T_map_odom =
        T_map_base *
        T_odom_base.inverse();


    // ============================================================
    // Publish map -> odom
    // ============================================================

    geometry_msgs::TransformStamped tf_msg;


    /*
     * IMPORTANT:
     *
     * This transform is built from the latest samples of two
     * independent approximately-10-Hz branches.
     *
     * Therefore publish it at the current ROS time rather than
     * reusing either measurement timestamp.
     *
     * This prevents the old map->odom transform from being inserted
     * several seconds in the past.
     */
    tf_msg.header.stamp =
        ros::Time::now();


    tf_msg.header.frame_id =
        map_frame_;


    tf_msg.child_frame_id =
        odom_frame_;


    tf_msg.transform =
        tf2::toMsg(
            T_map_odom);


    if (publish_tf_)
    {
      tf_broadcaster_.sendTransform(
          tf_msg);
    }


    have_global_pose_ =
        true;


    // ============================================================
    // Diagnostics
    // ============================================================

    double roll;
    double pitch;
    double yaw;


    tf2::Matrix3x3(
        T_map_odom.getRotation())
        .getRPY(
            roll,
            pitch,
            yaw);


    ROS_INFO_THROTTLE(
        2.0,
        "Publishing TF %s -> %s: "
        "xyz=[%.3f %.3f %.3f], "
        "rpy=[%.2f %.2f %.2f] deg",
        map_frame_.c_str(),
        odom_frame_.c_str(),
        T_map_odom.getOrigin().x(),
        T_map_odom.getOrigin().y(),
        T_map_odom.getOrigin().z(),
        roll * 180.0 / M_PI,
        pitch * 180.0 / M_PI,
        yaw * 180.0 / M_PI);
  }


private:

  ros::NodeHandle nh_;

  ros::NodeHandle pnh_;


  tf2_ros::TransformBroadcaster
      tf_broadcaster_;


  ros::Subscriber pose_sub_;

  ros::Subscriber odom_sub_;


  std::string global_pose_topic_;

  std::string odom_topic_;

  std::string map_frame_;

  std::string odom_frame_;

  std::string base_frame_;


  bool publish_tf_;

  double max_odom_age_sec_;


  bool have_odom_;

  bool have_global_pose_;


  nav_msgs::Odometry latest_odom_;


  std::mutex mutex_;
};


int main(
    int argc,
    char** argv)
{
  ros::init(
      argc,
      argv,
      "go2_relocalization_bridge");


  RelocalizationBridge node;


  ros::spin();


  return 0;
}