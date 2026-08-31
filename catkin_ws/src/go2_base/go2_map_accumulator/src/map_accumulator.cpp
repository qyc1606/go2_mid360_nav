#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <mutex>
#include <string>
#include <vector>

class MapAccumulator
{
public:
  MapAccumulator()
    : pnh_("~"),
      map_(new pcl::PointCloud<pcl::PointXYZI>)
  {
    pnh_.param<std::string>(
        "input_topic",
        input_topic_,
        "/cloud_registered_odom");

    pnh_.param<std::string>(
        "output_pcd",
        output_pcd_,
        "/tmp/public_map.pcd");

    pnh_.param<double>(
        "voxel_leaf",
        voxel_leaf_,
        0.05);

    pnh_.param<int>(
        "filter_every_n_frames",
        filter_every_n_frames_,
        10);

    pnh_.param<bool>(
        "save_on_shutdown",
        save_on_shutdown_,
        true);

    if (voxel_leaf_ <= 0.0) {
      ROS_WARN("voxel_leaf <= 0, reset to 0.05 m");
      voxel_leaf_ = 0.05;
    }

    if (filter_every_n_frames_ <= 0) {
      ROS_WARN("filter_every_n_frames <= 0, reset to 10");
      filter_every_n_frames_ = 10;
    }

    cloud_sub_ = nh_.subscribe(
        input_topic_,
        2,
        &MapAccumulator::cloudCallback,
        this);

    save_srv_ = pnh_.advertiseService(
        "save",
        &MapAccumulator::saveCallback,
        this);

    ROS_INFO("go2_map_accumulator started");
    ROS_INFO("input_topic: %s", input_topic_.c_str());
    ROS_INFO("output_pcd: %s", output_pcd_.c_str());
    ROS_INFO("voxel_leaf: %.3f m", voxel_leaf_);
    ROS_INFO("filter_every_n_frames: %d", filter_every_n_frames_);
    ROS_INFO("save_on_shutdown: %s",
             save_on_shutdown_ ? "true" : "false");
  }

  ~MapAccumulator()
  {
    if (save_on_shutdown_) {
      ROS_INFO("Shutdown requested, saving accumulated map...");
      saveMap(false);
    }
  }

private:
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZI>);

    try {
      pcl::fromROSMsg(*msg, *cloud);
    }
    catch (const std::exception& e) {
      ROS_ERROR_THROTTLE(
          2.0,
          "Failed to convert PointCloud2: %s",
          e.what());
      return;
    }

    if (cloud->empty()) {
      ROS_WARN_THROTTLE(2.0, "Received empty cloud");
      return;
    }

    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(
        *cloud,
        *cloud,
        valid_indices);

    if (cloud->empty()) {
      ROS_WARN_THROTTLE(
          2.0,
          "Cloud contains no valid points");
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);

      *map_ += *cloud;
      ++frame_count_;

      if (frame_count_ % filter_every_n_frames_ == 0) {
        applyVoxelFilterLocked();
      }

      if (frame_count_ % 100 == 0) {
        ROS_INFO(
            "Accumulated frames=%zu points=%zu",
            frame_count_,
            map_->size());
      }
    }
  }

  void applyVoxelFilterLocked()
  {
    if (map_->empty()) {
      return;
    }

    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setInputCloud(map_);

    const float leaf =
        static_cast<float>(voxel_leaf_);

    voxel.setLeafSize(
        leaf,
        leaf,
        leaf);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(
        new pcl::PointCloud<pcl::PointXYZI>);

    voxel.filter(*filtered);

    map_.swap(filtered);
  }

  bool saveCallback(
      std_srvs::Trigger::Request&,
      std_srvs::Trigger::Response& res)
  {
    std::string message;

    const bool ok = saveMap(
        true,
        &message);

    res.success = ok;
    res.message = message;

    return true;
  }

  bool saveMap(
      bool report_empty_as_error,
      std::string* result_message = nullptr)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (map_->empty()) {
      const std::string msg =
          "map is empty; nothing to save";

      if (result_message) {
        *result_message = msg;
      }

      if (report_empty_as_error) {
        ROS_ERROR("%s", msg.c_str());
      }
      else {
        ROS_WARN("%s", msg.c_str());
      }

      return false;
    }

    // 保存前再做一次最终体素滤波。
    applyVoxelFilterLocked();

    const int ret =
        pcl::io::savePCDFileBinary(
            output_pcd_,
            *map_);

    if (ret != 0) {
      const std::string msg =
          "failed to save PCD: " + output_pcd_;

      if (result_message) {
        *result_message = msg;
      }

      ROS_ERROR("%s", msg.c_str());
      return false;
    }

    const std::string msg =
        "saved " +
        output_pcd_ +
        " points=" +
        std::to_string(map_->size());

    if (result_message) {
      *result_message = msg;
    }

    ROS_INFO("%s", msg.c_str());

    return true;
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber cloud_sub_;
  ros::ServiceServer save_srv_;

  std::string input_topic_;
  std::string output_pcd_;

  double voxel_leaf_{0.05};
  int filter_every_n_frames_{10};
  bool save_on_shutdown_{true};

  std::size_t frame_count_{0};

  pcl::PointCloud<pcl::PointXYZI>::Ptr map_;

  std::mutex mutex_;
};


int main(int argc, char** argv)
{
  ros::init(
      argc,
      argv,
      "go2_map_accumulator");

  MapAccumulator node;

  ros::spin();

  return 0;
}



