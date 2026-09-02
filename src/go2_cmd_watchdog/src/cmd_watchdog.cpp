#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

#include <algorithm>
#include <cmath>

class CmdWatchdog
{
public:
  CmdWatchdog()
      : nh_(),
        pnh_("~"),
        localization_ok_(false),
        have_localization_status_(false),
        have_command_(false)
  {
    pnh_.param<double>(
        "cmd_timeout_sec",
        command_timeout_sec_,
        0.50);

    pnh_.param<double>(
        "max_vx",
        max_vx_,
        0.20);

    pnh_.param<double>(
        "max_vy",
        max_vy_,
        0.00);

    pnh_.param<double>(
        "max_wz",
        max_wz_,
        0.30);

    pnh_.param<double>(
        "localization_timeout_sec",
        localization_timeout_sec_,
        0.75);

    command_sub_ = nh_.subscribe(
        "/cmd_vel_nav",
        10,
        &CmdWatchdog::commandCallback,
        this);

    localization_sub_ = nh_.subscribe(
        "/localization/ok",
        10,
        &CmdWatchdog::localizationCallback,
        this);

    safe_command_pub_ =
        nh_.advertise<geometry_msgs::Twist>(
            "/cmd_vel_safe",
            10);

    timer_ = nh_.createTimer(
        ros::Duration(0.05),
        &CmdWatchdog::timerCallback,
        this);
  }

private:
  static double clamp(
      double value,
      double limit)
  {
    return std::max(
        -limit,
        std::min(
            limit,
            value));
  }

  static bool finiteTwist(
      const geometry_msgs::Twist& msg)
  {
    return
        std::isfinite(msg.linear.x) &&
        std::isfinite(msg.linear.y) &&
        std::isfinite(msg.linear.z) &&
        std::isfinite(msg.angular.x) &&
        std::isfinite(msg.angular.y) &&
        std::isfinite(msg.angular.z);
  }

  void commandCallback(
      const geometry_msgs::Twist::ConstPtr& msg)
  {
    if (!finiteTwist(*msg))
    {
      have_command_ = false;
      last_command_ = geometry_msgs::Twist();
      ROS_ERROR_THROTTLE(
          1.0,
          "Rejected non-finite navigation velocity command.");
      return;
    }

    last_command_ = *msg;
    last_command_wall_time_ = ros::WallTime::now();
    have_command_ = true;
  }

  void localizationCallback(
      const std_msgs::Bool::ConstPtr& msg)
  {
    localization_ok_ = msg->data;
    last_localization_wall_time_ = ros::WallTime::now();
    have_localization_status_ = true;
  }

  void timerCallback(
      const ros::TimerEvent&)
  {
    geometry_msgs::Twist output;

    bool safe = true;

    const ros::WallTime now = ros::WallTime::now();

    if (!have_localization_status_ || !localization_ok_)
    {
      safe = false;
    }

    if (have_localization_status_)
    {
      const double localization_age_sec =
          (now - last_localization_wall_time_).toSec();

      if (localization_age_sec > localization_timeout_sec_)
      {
        safe = false;
      }
    }

    if (!have_command_)
    {
      safe = false;
    }

    if (have_command_)
    {
      const double age_sec =
          (now -
           last_command_wall_time_).toSec();

      if (age_sec > command_timeout_sec_)
      {
        safe = false;
      }
    }

    if (safe)
    {
      output.linear.x =
          clamp(
              last_command_.linear.x,
              max_vx_);

      output.linear.y =
          clamp(
              last_command_.linear.y,
              max_vy_);

      output.angular.z =
          clamp(
              last_command_.angular.z,
              max_wz_);
    }

    safe_command_pub_.publish(output);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber command_sub_;
  ros::Subscriber localization_sub_;

  ros::Publisher safe_command_pub_;

  ros::Timer timer_;

  geometry_msgs::Twist last_command_;
  ros::WallTime last_command_wall_time_;
  ros::WallTime last_localization_wall_time_;

  bool localization_ok_;
  bool have_localization_status_;
  bool have_command_;

  double command_timeout_sec_;
  double max_vx_;
  double max_vy_;
  double max_wz_;
  double localization_timeout_sec_;
};

int main(
    int argc,
    char** argv)
{
  ros::init(
      argc,
      argv,
      "go2_cmd_watchdog");

  CmdWatchdog node;

  ros::spin();

  return 0;
}
