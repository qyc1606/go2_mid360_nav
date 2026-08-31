#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <string>

class WaypointManager
{
public:
  WaypointManager()
      : nh_(),
        pnh_("~"),
        tf_buffer_(),
        tf_listener_(tf_buffer_),
        localization_ok_(false),
        have_path_(false),
        current_index_(0)
  {
    pnh_.param<std::string>(
        "input_path_topic",
        input_path_topic_,
        "/sparse_waypoints");

    pnh_.param<std::string>(
        "goal_topic",
        goal_topic_,
        "/ego/goal");

    pnh_.param<std::string>(
        "odom_frame",
        odom_frame_,
        "odom");

    pnh_.param<std::string>(
        "base_frame",
        base_frame_,
        "base_link");

    pnh_.param<double>(
        "reach_distance_m",
        reach_distance_m_,
        0.50);

    path_sub_ = nh_.subscribe(
        input_path_topic_,
        1,
        &WaypointManager::pathCallback,
        this);

    localization_sub_ = nh_.subscribe(
        "/localization/ok",
        10,
        &WaypointManager::localizationCallback,
        this);

    goal_pub_ =
        nh_.advertise<geometry_msgs::PoseStamped>(
            goal_topic_,
            1,
            true);

    timer_ = nh_.createTimer(
        ros::Duration(0.2),
        &WaypointManager::timerCallback,
        this);
  }

private:
  void localizationCallback(
      const std_msgs::Bool::ConstPtr& msg)
  {
    localization_ok_ = msg->data;
  }

  void pathCallback(
      const nav_msgs::Path::ConstPtr& msg)
  {
    path_ = *msg;
    current_index_ = 0;
    have_path_ = !path_.poses.empty();

    ROS_INFO(
        "Received %zu sparse waypoints",
        path_.poses.size());
  }

  void timerCallback(
      const ros::TimerEvent&)
  {
    if (!localization_ok_ ||
        !have_path_)
    {
      return;
    }

    if (current_index_ >= path_.poses.size())
    {
      have_path_ = false;
      return;
    }

    geometry_msgs::TransformStamped
        map_to_odom;

    geometry_msgs::TransformStamped
        odom_to_base;

    try
    {
      map_to_odom =
          tf_buffer_.lookupTransform(
              odom_frame_,
              path_.header.frame_id,
              ros::Time(0),
              ros::Duration(0.2));

      odom_to_base =
          tf_buffer_.lookupTransform(
              odom_frame_,
              base_frame_,
              ros::Time(0),
              ros::Duration(0.2));
    }
    catch (const tf2::TransformException& e)
    {
      ROS_WARN_THROTTLE(
          1.0,
          "%s",
          e.what());

      return;
    }

    geometry_msgs::PoseStamped target_odom;

    tf2::doTransform(
        path_.poses[current_index_],
        target_odom,
        map_to_odom);

    const double dx =
        target_odom.pose.position.x -
        odom_to_base.transform.translation.x;

    const double dy =
        target_odom.pose.position.y -
        odom_to_base.transform.translation.y;

    const double distance =
        std::sqrt(
            dx * dx +
            dy * dy);

    if (distance < reach_distance_m_)
    {
      current_index_++;

      if (current_index_ >= path_.poses.size())
      {
        have_path_ = false;

        ROS_INFO(
            "Waypoint sequence completed");

        return;
      }

      tf2::doTransform(
          path_.poses[current_index_],
          target_odom,
          map_to_odom);
    }

    target_odom.header.stamp =
        ros::Time::now();

    target_odom.header.frame_id =
        odom_frame_;

    goal_pub_.publish(
        target_odom);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber path_sub_;
  ros::Subscriber localization_sub_;

  ros::Publisher goal_pub_;

  ros::Timer timer_;

  nav_msgs::Path path_;

  std::string input_path_topic_;
  std::string goal_topic_;
  std::string odom_frame_;
  std::string base_frame_;

  bool localization_ok_;
  bool have_path_;

  size_t current_index_;

  double reach_distance_m_;
};

int main(
    int argc,
    char** argv)
{
  ros::init(
      argc,
      argv,
      "go2_waypoint_manager");

  WaypointManager node;

  ros::spin();

  return 0;
}
