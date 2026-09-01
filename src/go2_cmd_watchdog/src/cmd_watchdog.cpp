#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

#include <algorithm>

class CmdWatchdog
{
public:
  CmdWatchdog()
      : nh_(),
        pnh_("~"),
        localization_ok_(false),
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
        0.10);

    pnh_.param<double>(
        "max_wz",
        max_wz_,
        0.30);

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

  void commandCallback(
      const geometry_msgs::Twist::ConstPtr& msg)
  {
    last_command_ = *msg;
    last_command_wall_time_ = ros::WallTime::now();
    have_command_ = true;
  }

  void localizationCallback(
      const std_msgs::Bool::ConstPtr& msg)
  {
    localization_ok_ = msg->data;
  }

  void timerCallback(
      const ros::TimerEvent&)
  {
    geometry_msgs::Twist output;

    bool safe = true;

    if (!localization_ok_)
    {
      safe = false;
    }

    if (!have_command_)
    {
      safe = false;
    }

    if (have_command_)
    {
      const double age_sec =
          (ros::WallTime::now() -
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

  bool localization_ok_;
  bool have_command_;

  double command_timeout_sec_;
  double max_vx_;
  double max_vy_;
  double max_wz_;
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
