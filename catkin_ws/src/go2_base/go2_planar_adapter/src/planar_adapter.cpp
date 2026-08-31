#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

/*
 * V8 compatibility node.
 *
 * go2_pose_adapter is now the ONLY owner of public robot TF:
 *   odom -> base_footprint -> base_link
 *
 * This package is retained only so old scripts do not break.
 * It republishes /odom_nav to /odom_nav_legacy and NEVER broadcasts TF.
 */

class PlanarAdapterCompat
{
public:
  PlanarAdapterCompat()
    : nh_(), pnh_("~")
  {
    pnh_.param<std::string>("input_topic", input_topic_, "/odom_nav");
    pnh_.param<std::string>("output_topic", output_topic_, "/odom_nav_legacy");

    pub_ = nh_.advertise<nav_msgs::Odometry>(output_topic_, 10);
    sub_ = nh_.subscribe(input_topic_, 20, &PlanarAdapterCompat::cb, this);

    ROS_WARN("go2_planar_adapter is compatibility-only in V8. "
             "Public TF and /odom_nav are owned by go2_pose_adapter.");
  }

private:
  void cb(const nav_msgs::Odometry::ConstPtr& msg)
  {
    pub_.publish(*msg);
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  std::string input_topic_, output_topic_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "go2_planar_adapter");
  PlanarAdapterCompat node;
  ros::spin();
  return 0;
}
