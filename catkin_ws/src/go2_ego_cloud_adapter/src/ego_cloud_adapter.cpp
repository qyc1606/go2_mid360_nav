#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <string>

class EgoCloudAdapter
{
public:
  EgoCloudAdapter()
      : nh_(),
        pnh_("~")
  {
    pnh_.param<std::string>(
        "input_topic",
        input_topic_,
        "/cloud_registered_odom");

    pnh_.param<std::string>(
        "output_topic",
        output_topic_,
        "/ego/cloud");

    pnh_.param<double>(
        "voxel_leaf",
        voxel_leaf_,
        0.08);

    pnh_.param<double>(
        "min_z",
        min_z_,
        0.08);

    pnh_.param<double>(
        "max_z",
        max_z_,
        0.70);

    pnh_.param<double>(
        "extrude_z_min",
        extrude_z_min_,
        0.0);

    pnh_.param<double>(
        "extrude_z_max",
        extrude_z_max_,
        1.0);

    pnh_.param<double>(
        "extrude_step",
        extrude_step_,
        0.10);

    cloud_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>(
            output_topic_,
            2);

    cloud_sub_ = nh_.subscribe(
        input_topic_,
        2,
        &EgoCloudAdapter::cloudCallback,
        this);
  }

private:
  void cloudCallback(
      const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    pcl::PointCloud<pcl::PointXYZI> input;

    pcl::fromROSMsg(
        *msg,
        input);

    pcl::PointCloud<pcl::PointXYZI>::Ptr
        height_filtered(
            new pcl::PointCloud<pcl::PointXYZI>());

    height_filtered->reserve(
        input.size());

    for (const auto& point : input.points)
    {
      if (!std::isfinite(point.x) ||
          !std::isfinite(point.y) ||
          !std::isfinite(point.z))
      {
        continue;
      }

      if (point.z < min_z_ ||
          point.z > max_z_)
      {
        continue;
      }

      height_filtered->push_back(point);
    }

    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;

    voxel_filter.setInputCloud(
        height_filtered);

    const float leaf =
        static_cast<float>(voxel_leaf_);

    voxel_filter.setLeafSize(
        leaf,
        leaf,
        leaf);

    pcl::PointCloud<pcl::PointXYZI> voxel_cloud;

    voxel_filter.filter(voxel_cloud);

    pcl::PointCloud<pcl::PointXYZI> output;

    for (const auto& point : voxel_cloud.points)
    {
      for (double z = extrude_z_min_;
           z <= extrude_z_max_ + 1e-6;
           z += extrude_step_)
      {
        pcl::PointXYZI p = point;
        p.z = static_cast<float>(z);

        output.push_back(p);
      }
    }

    output.header = voxel_cloud.header;

    sensor_msgs::PointCloud2 ros_output;

    pcl::toROSMsg(
        output,
        ros_output);

    ros_output.header = msg->header;

    cloud_pub_.publish(
        ros_output);

    ROS_INFO_THROTTLE(
        2.0,
        "EGO cloud: input=%zu filtered=%zu voxel=%zu output=%zu",
        input.size(),
        height_filtered->size(),
        voxel_cloud.size(),
        output.size());
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber cloud_sub_;
  ros::Publisher cloud_pub_;

  std::string input_topic_;
  std::string output_topic_;

  double voxel_leaf_;
  double min_z_;
  double max_z_;

  double extrude_z_min_;
  double extrude_z_max_;
  double extrude_step_;
};

int main(
    int argc,
    char** argv)
{
  ros::init(
      argc,
      argv,
      "go2_ego_cloud_adapter");

  EgoCloudAdapter node;

  ros::spin();

  return 0;
}
