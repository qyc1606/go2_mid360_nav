#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>


class Go2SdkBridgeReal
{
public:
  Go2SdkBridgeReal()
      : nh_(),
        pnh_("~"),
        enabled_(false),
        localization_ok_(false),
        have_cmd_(false)
  {
    // ============================================================
    // Parameters
    // ============================================================

    pnh_.param<std::string>(
        "command_topic",
        command_topic_,
        "/cmd_vel_safe");

    pnh_.param<std::string>(
        "localization_ok_topic",
        localization_ok_topic_,
        "/localization/ok");

    pnh_.param<std::string>(
        "network_interface",
        network_interface_,
        "eth1");

    pnh_.param<double>(
        "command_timeout_sec",
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
        "min_walk_vx",
        min_walk_vx_,
        0.25);

    pnh_.param<double>(
        "stop_deadband_vx",
        stop_deadband_vx_,
        0.03);

    pnh_.param<double>(
        "stop_deadband_wz",
        stop_deadband_wz_,
        0.03);

    pnh_.param<double>(
        "control_rate_hz",
        control_rate_hz_,
        50.0);


    // ============================================================
    // Unitree SDK2 initialization
    // ============================================================

    ROS_WARN(
        "STEP A: initializing Unitree ChannelFactory on [%s]",
        network_interface_.c_str());

    unitree::robot::ChannelFactory::Instance()->Init(
        0,
        network_interface_);

    ROS_WARN(
        "STEP B: Unitree ChannelFactory initialized");


    sport_client_.reset(
        new unitree::robot::go2::SportClient());

    ROS_WARN(
        "STEP C: Unitree SportClient constructed");


    sport_client_->SetTimeout(
        1.0f);

    sport_client_->Init();

    ROS_WARN(
        "STEP D: Unitree SportClient initialized");


    // ============================================================
    // ROS
    // ============================================================

    command_sub_ =
        nh_.subscribe(
            command_topic_,
            10,
            &Go2SdkBridgeReal::commandCallback,
            this);

    localization_sub_ =
        nh_.subscribe(
            localization_ok_topic_,
            10,
            &Go2SdkBridgeReal::localizationCallback,
            this);

    enable_service_ =
        pnh_.advertiseService(
            "enable",
            &Go2SdkBridgeReal::enableCallback,
            this);

    control_timer_ =
        nh_.createTimer(
            ros::Duration(
                1.0 /
                std::max(
                    1.0,
                    control_rate_hz_)),
            &Go2SdkBridgeReal::controlCallback,
            this);


    stopRobot();


    ROS_WARN(
        "REAL GO2 SDK bridge started DISABLED.");

    ROS_INFO(
        "input=%s interface=%s",
        command_topic_.c_str(),
        network_interface_.c_str());

    ROS_INFO(
        "limits: vx=%.3f vy=%.3f wz=%.3f",
        max_vx_,
        max_vy_,
        max_wz_);

    ROS_INFO(
        "walk threshold: deadband=%.3f min_walk=%.3f",
        stop_deadband_vx_,
        min_walk_vx_);

    ROS_INFO(
        "timeout=%.3f sec control_rate=%.1f Hz",
        command_timeout_sec_,
        control_rate_hz_);
  }


  ~Go2SdkBridgeReal()
  {
    enabled_.store(false);

    stopRobot();

    ROS_WARN(
        "GO2 SDK bridge shutdown: StopMove sent.");
  }


private:

  static double clampValue(
      double value,
      double limit)
  {
    if (limit <= 0.0)
    {
      return 0.0;
    }

    return std::max(
        -limit,
        std::min(
            limit,
            value));
  }


  static bool finiteTwist(
      const geometry_msgs::Twist& cmd)
  {
    return
        std::isfinite(cmd.linear.x) &&
        std::isfinite(cmd.linear.y) &&
        std::isfinite(cmd.linear.z) &&
        std::isfinite(cmd.angular.x) &&
        std::isfinite(cmd.angular.y) &&
        std::isfinite(cmd.angular.z);
  }


  double shapeForwardVelocity(
      double vx) const
  {
    const double abs_vx =
        std::fabs(vx);


    if (abs_vx <
        stop_deadband_vx_)
    {
      return 0.0;
    }


    const double magnitude =
        std::max(
            abs_vx,
            min_walk_vx_);


    const double limited =
        std::min(
            magnitude,
            max_vx_);


    return
        std::copysign(
            limited,
            vx);
  }


  double shapeYawRate(
      double wz) const
  {
    if (std::fabs(wz) <
        stop_deadband_wz_)
    {
      return 0.0;
    }


    return
        clampValue(
            wz,
            max_wz_);
  }


  void localizationCallback(
      const std_msgs::Bool::ConstPtr& msg)
  {
    const bool previous =
        localization_ok_.load();

    localization_ok_.store(
        msg->data);


    if (previous &&
        !msg->data)
    {
      ROS_ERROR(
          "Localization changed from OK to NOT OK.");

      enabled_.store(false);

      stopRobot();
    }
  }


  void commandCallback(
      const geometry_msgs::Twist::ConstPtr& msg)
  {
    if (!finiteTwist(*msg))
    {
      ROS_ERROR_THROTTLE(
          1.0,
          "Rejected non-finite velocity command.");

      return;
    }


    {
      std::lock_guard<std::mutex>
          lock(cmd_mutex_);

      last_cmd_ =
          *msg;

      last_cmd_wall_stamp_ =
          ros::WallTime::now();

      have_cmd_ =
          true;
    }


    ROS_INFO_THROTTLE(
        1.0,
        "CMD RX: vx=%.3f vy=%.3f wz=%.3f",
        msg->linear.x,
        msg->linear.y,
        msg->angular.z);
  }


  bool enableCallback(
      std_srvs::SetBool::Request& request,
      std_srvs::SetBool::Response& response)
  {
    if (!request.data)
    {
      enabled_.store(false);

      stopRobot();

      response.success =
          true;

      response.message =
          "REAL GO2 SDK bridge disabled; StopMove sent.";

      ROS_WARN(
          "REAL GO2 motion bridge DISABLED.");

      return true;
    }


    if (!localization_ok_.load())
    {
      enabled_.store(false);

      stopRobot();

      response.success =
          false;

      response.message =
          "Cannot enable: localization/ok is false.";

      return true;
    }


    bool have_cmd = false;

    ros::WallTime cmd_stamp;


    {
      std::lock_guard<std::mutex>
          lock(cmd_mutex_);

      have_cmd =
          have_cmd_;

      cmd_stamp =
          last_cmd_wall_stamp_;
    }


    if (!have_cmd)
    {
      enabled_.store(false);

      stopRobot();

      response.success =
          false;

      response.message =
          "Cannot enable: no velocity command received.";

      return true;
    }


    const double age =
        (ros::WallTime::now() -
         cmd_stamp).toSec();


    if (age >
        command_timeout_sec_)
    {
      enabled_.store(false);

      stopRobot();

      response.success =
          false;

      response.message =
          "Cannot enable: latest velocity command is stale.";

      ROS_ERROR(
          "Enable rejected: command age %.3f > %.3f sec",
          age,
          command_timeout_sec_);

      return true;
    }


    enabled_.store(true);

    response.success =
        true;

    response.message =
        "REAL GO2 SDK bridge ENABLED.";

    ROS_WARN(
        "REAL GO2 motion bridge ENABLED.");

    return true;
  }


  void controlCallback(
      const ros::TimerEvent&)
  {
    if (!sport_client_)
    {
      return;
    }


    if (!enabled_.load())
    {
      stopRobotThrottled();

      return;
    }


    if (!localization_ok_.load())
    {
      enabled_.store(false);

      stopRobot();

      ROS_ERROR_THROTTLE(
          1.0,
          "Localization lost. Bridge disabled.");

      return;
    }


    geometry_msgs::Twist cmd;

    ros::WallTime cmd_stamp;

    bool have_cmd = false;


    {
      std::lock_guard<std::mutex>
          lock(cmd_mutex_);

      have_cmd =
          have_cmd_;

      cmd =
          last_cmd_;

      cmd_stamp =
          last_cmd_wall_stamp_;
    }


    if (!have_cmd)
    {
      stopRobot();

      return;
    }


    const double age =
        (ros::WallTime::now() -
         cmd_stamp).toSec();


    if (age >
        command_timeout_sec_)
    {
      stopRobot();

      ROS_WARN_THROTTLE(
          1.0,
          "Velocity command timeout: %.3f > %.3f sec",
          age,
          command_timeout_sec_);

      return;
    }


    // ============================================================
    // GO2 velocity shaping
    // ============================================================

    const double vx =
        shapeForwardVelocity(
            cmd.linear.x);

    // First real navigation stage:
    // lateral motion disabled.
    const double vy =
        0.0;

    const double wz =
        shapeYawRate(
            cmd.angular.z);


    // If both forward and yaw commands are effectively zero,
    // use StopMove instead of sending tiny Move commands.
    if (std::fabs(vx) < 1e-6 &&
        std::fabs(wz) < 1e-6)
    {
      stopRobot();

      ROS_INFO_THROTTLE(
          1.0,
          "GO2 command in stop deadband -> StopMove.");

      return;
    }


    sport_client_->Move(
        vx,
        vy,
        wz);


    ROS_INFO_THROTTLE(
        0.5,
        "GO2 MOVE raw=(%.3f %.3f %.3f) shaped=(%.3f %.3f %.3f)",
        cmd.linear.x,
        cmd.linear.y,
        cmd.angular.z,
        vx,
        vy,
        wz);
  }


  void stopRobot()
  {
    if (sport_client_)
    {
      sport_client_->StopMove();
    }
  }


  void stopRobotThrottled()
  {
    if (sport_client_)
    {
      sport_client_->StopMove();
    }


    ROS_INFO_THROTTLE(
        2.0,
        "Bridge disabled: StopMove.");
  }


private:

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber command_sub_;
  ros::Subscriber localization_sub_;

  ros::ServiceServer enable_service_;

  ros::Timer control_timer_;


  std::unique_ptr<
      unitree::robot::go2::SportClient>
      sport_client_;


  std::atomic<bool>
      enabled_;

  std::atomic<bool>
      localization_ok_;


  std::mutex cmd_mutex_;

  geometry_msgs::Twist
      last_cmd_;

  ros::WallTime
      last_cmd_wall_stamp_;

  bool
      have_cmd_;


  std::string
      command_topic_;

  std::string
      localization_ok_topic_;

  std::string
      network_interface_;


  double
      command_timeout_sec_;

  double
      max_vx_;

  double
      max_vy_;

  double
      max_wz_;

  double
      min_walk_vx_;

  double
      stop_deadband_vx_;

  double
      stop_deadband_wz_;

  double
      control_rate_hz_;
};


int main(
    int argc,
    char** argv)
{
  ros::init(
      argc,
      argv,
      "go2_sdk_bridge_real");


  ROS_WARN(
      "Starting REAL Unitree GO2 SDK bridge.");


  Go2SdkBridgeReal node;


  ros::spin();


  return 0;
}
