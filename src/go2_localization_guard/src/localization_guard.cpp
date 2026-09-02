#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <string>

class LocalizationGuard
{
public:
  LocalizationGuard()
      : nh_(),
        pnh_("~"),
        have_prev_(false),
        have_score_(false),
        have_iterations_(false),
        have_translation_jump_(false),
        have_rotation_jump_(false),
        have_last_success_age_(false),
        good_count_(0),
        bad_count_(0),
        current_state_("INIT"),
        current_ok_(false),
        current_confidence_(0.0f)
  {
    pnh_.param<std::string>(
        "global_pose_topic",
        global_pose_topic_,
        "/localization");

    pnh_.param<double>(
        "timeout_sec",
        timeout_sec_,
        2.0);

    pnh_.param<double>(
        "max_jump_m",
        max_jump_m_,
        0.8);

    pnh_.param<double>(
        "max_jump_yaw_deg",
        max_jump_yaw_deg_,
        20.0);

    pnh_.param<int>(
        "good_required_count",
        good_required_count_,
        5);

    pnh_.param<int>(
        "lost_required_count",
        lost_required_count_,
        3);

    pnh_.param<double>("max_ndt_score", max_ndt_score_, 2.0);
    pnh_.param<int>("max_ndt_iterations", max_ndt_iterations_, 20);
    pnh_.param<double>(
        "max_last_success_age_sec",
        max_last_success_age_sec_,
        2.0);

    pose_sub_ = nh_.subscribe(
        global_pose_topic_,
        10,
        &LocalizationGuard::poseCallback,
        this);

    score_sub_ = nh_.subscribe(
        "/localization/ndt_score", 10,
        &LocalizationGuard::scoreCallback, this);
    iterations_sub_ = nh_.subscribe(
        "/localization/ndt_iterations", 10,
        &LocalizationGuard::iterationsCallback, this);
    translation_jump_sub_ = nh_.subscribe(
        "/localization/translation_jump", 10,
        &LocalizationGuard::translationJumpCallback, this);
    rotation_jump_sub_ = nh_.subscribe(
        "/localization/rotation_jump", 10,
        &LocalizationGuard::rotationJumpCallback, this);
    last_success_age_sub_ = nh_.subscribe(
        "/localization/last_success_age", 10,
        &LocalizationGuard::lastSuccessAgeCallback, this);

    state_pub_ = nh_.advertise<std_msgs::String>(
        "/localization/state",
        10,
        true);

    ok_pub_ = nh_.advertise<std_msgs::Bool>(
        "/localization/ok",
        10,
        true);

    confidence_pub_ =
        nh_.advertise<std_msgs::Float32>(
            "/localization/confidence",
            10,
            true);

    timer_ = nh_.createTimer(
        ros::Duration(0.2),
        &LocalizationGuard::timerCallback,
        this);

    publishState(
        "INIT",
        false,
        0.0f);
  }

private:
  static bool finitePose(const geometry_msgs::Pose& pose)
  {
    const double q_norm_squared =
        pose.orientation.x * pose.orientation.x +
        pose.orientation.y * pose.orientation.y +
        pose.orientation.z * pose.orientation.z +
        pose.orientation.w * pose.orientation.w;

    return
        std::isfinite(pose.position.x) &&
        std::isfinite(pose.position.y) &&
        std::isfinite(pose.position.z) &&
        std::isfinite(pose.orientation.x) &&
        std::isfinite(pose.orientation.y) &&
        std::isfinite(pose.orientation.z) &&
        std::isfinite(pose.orientation.w) &&
        std::isfinite(q_norm_squared) &&
        q_norm_squared > 1e-12;
  }

  static double wrapAngle(double a)
  {
    while (a > M_PI)
    {
      a -= 2.0 * M_PI;
    }

    while (a < -M_PI)
    {
      a += 2.0 * M_PI;
    }

    return a;
  }

  static double yawFromQuaternion(
      const geometry_msgs::Quaternion& qmsg)
  {
    tf2::Quaternion q(
        qmsg.x,
        qmsg.y,
        qmsg.z,
        qmsg.w);

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    tf2::Matrix3x3(q).getRPY(
        roll,
        pitch,
        yaw);

    return yaw;
  }

  void publishState(
      const std::string& state,
      bool ok,
      float confidence)
  {
    current_state_ = state;
    current_ok_ = ok;
    current_confidence_ = confidence;

    std_msgs::String state_msg;
    state_msg.data = state;
    state_pub_.publish(state_msg);

    std_msgs::Bool ok_msg;
    ok_msg.data = ok;
    ok_pub_.publish(ok_msg);

    std_msgs::Float32 confidence_msg;
    confidence_msg.data = confidence;
    confidence_pub_.publish(confidence_msg);
  }

  void publishCurrentState()
  {
    publishState(
        current_state_,
        current_ok_,
        current_confidence_);
  }

  void scoreCallback(const std_msgs::Float64::ConstPtr& msg)
  {
    ndt_score_ = msg->data;
    have_score_ = true;
  }

  void iterationsCallback(const std_msgs::Int32::ConstPtr& msg)
  {
    ndt_iterations_ = msg->data;
    have_iterations_ = true;
  }

  void translationJumpCallback(const std_msgs::Float64::ConstPtr& msg)
  {
    translation_jump_ = msg->data;
    have_translation_jump_ = true;
  }

  void rotationJumpCallback(const std_msgs::Float64::ConstPtr& msg)
  {
    rotation_jump_ = msg->data;
    have_rotation_jump_ = true;
  }

  void lastSuccessAgeCallback(const std_msgs::Float64::ConstPtr& msg)
  {
    last_success_age_ = msg->data;
    have_last_success_age_ = true;
  }

  bool qualityGood() const
  {
    return
        have_score_ &&
        have_iterations_ &&
        have_translation_jump_ &&
        have_rotation_jump_ &&
        have_last_success_age_ &&
        std::isfinite(ndt_score_) &&
        std::isfinite(translation_jump_) &&
        std::isfinite(rotation_jump_) &&
        std::isfinite(last_success_age_) &&
        ndt_score_ <= max_ndt_score_ &&
        ndt_iterations_ >= 0 &&
        ndt_iterations_ <= max_ndt_iterations_ &&
        translation_jump_ <= max_jump_m_ &&
        rotation_jump_ <= max_jump_yaw_deg_ * M_PI / 180.0 &&
        last_success_age_ <= max_last_success_age_sec_;
  }

  void poseCallback(
      const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    if (!finitePose(msg->pose))
    {
      good_count_ = 0;
      bad_count_ = lost_required_count_;
      publishState(
          "LOST",
          false,
          0.0f);
      ROS_ERROR_THROTTLE(
          1.0,
          "Rejected non-finite localization pose");
      return;
    }

    bool sample_good = qualityGood();

    if (have_prev_)
    {
      const double dx =
          msg->pose.position.x -
          previous_pose_.pose.position.x;

      const double dy =
          msg->pose.position.y -
          previous_pose_.pose.position.y;

      const double dz =
          msg->pose.position.z -
          previous_pose_.pose.position.z;

      const double distance =
          std::sqrt(
              dx * dx +
              dy * dy +
              dz * dz);

      const double yaw_now =
          yawFromQuaternion(
              msg->pose.orientation);

      const double yaw_previous =
          yawFromQuaternion(
              previous_pose_.pose.orientation);

      const double yaw_jump_deg =
          std::fabs(
              wrapAngle(
                  yaw_now -
                  yaw_previous)) *
          180.0 / M_PI;

      if (distance > max_jump_m_)
      {
        sample_good = false;

        ROS_WARN(
            "Localization jump %.3f m > %.3f m",
            distance,
            max_jump_m_);
      }

      if (yaw_jump_deg > max_jump_yaw_deg_)
      {
        sample_good = false;

        ROS_WARN(
            "Localization yaw jump %.2f deg > %.2f deg",
            yaw_jump_deg,
            max_jump_yaw_deg_);
      }
    }

    previous_pose_ = *msg;
    last_pose_wall_time_ = ros::WallTime::now();
    have_prev_ = true;

    if (sample_good)
    {
      good_count_++;
      bad_count_ = 0;
    }
    else
    {
      good_count_ = 0;
      bad_count_++;
    }

    if (bad_count_ >= lost_required_count_)
    {
      publishState(
          "LOST",
          false,
          0.0f);
    }
    else if (good_count_ >= good_required_count_)
    {
      publishState(
          "GOOD",
          true,
          1.0f);
    }
    else
    {
      publishState(
          "INIT",
          false,
          0.5f);
    }
  }

  void timerCallback(
      const ros::TimerEvent&)
  {
    if (!have_prev_)
    {
      publishState(
          "INIT",
          false,
          0.0f);

      return;
    }

    const double age_sec =
        (ros::WallTime::now() -
         last_pose_wall_time_).toSec();

    if (age_sec > timeout_sec_)
    {
      good_count_ = 0;
      bad_count_++;

      publishState(
          "LOST",
          false,
          0.0f);

      return;
    }

    if (!qualityGood())
    {
      good_count_ = 0;
      publishState(
          "LOST",
          false,
          0.0f);
      return;
    }

    publishCurrentState();
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber pose_sub_;
  ros::Subscriber score_sub_;
  ros::Subscriber iterations_sub_;
  ros::Subscriber translation_jump_sub_;
  ros::Subscriber rotation_jump_sub_;
  ros::Subscriber last_success_age_sub_;

  ros::Publisher state_pub_;
  ros::Publisher ok_pub_;
  ros::Publisher confidence_pub_;

  ros::Timer timer_;

  std::string global_pose_topic_;

  double timeout_sec_;
  double max_jump_m_;
  double max_jump_yaw_deg_;
  double max_ndt_score_;
  double max_last_success_age_sec_;

  int good_required_count_;
  int lost_required_count_;
  int max_ndt_iterations_;

  bool have_prev_;
  bool have_score_;
  bool have_iterations_;
  bool have_translation_jump_;
  bool have_rotation_jump_;
  bool have_last_success_age_;
  int good_count_;
  int bad_count_;

  double ndt_score_ = 0.0;
  int ndt_iterations_ = 0;
  double translation_jump_ = 0.0;
  double rotation_jump_ = 0.0;
  double last_success_age_ = 0.0;

  std::string current_state_;
  bool current_ok_;
  float current_confidence_;

  geometry_msgs::PoseStamped previous_pose_;
  ros::WallTime last_pose_wall_time_;
};

int main(
    int argc,
    char** argv)
{
  ros::init(
      argc,
      argv,
      "go2_localization_guard");

  LocalizationGuard node;

  ros::spin();

  return 0;
}
