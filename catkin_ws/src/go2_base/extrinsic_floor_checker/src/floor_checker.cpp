
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>



#include <pcl/point_cloud.h>

#include <pcl/point_types.h>

#include <pcl/filters/crop_box.h>

#include <pcl/segmentation/sac_segmentation.h>

#include <pcl_conversions/pcl_conversions.h>



#include <cmath>



class FloorChecker

{

public:

  FloorChecker()

    : nh_(),

      pnh_("~")

  {

    pnh_.param<std::string>(

      "cloud_topic",

      cloud_topic_,

      "/cloud_registered_base");



    pnh_.param<double>("min_x", min_x_, -1.5);

    pnh_.param<double>("max_x", max_x_,  2.0);

    pnh_.param<double>("min_y", min_y_, -1.5);

    pnh_.param<double>("max_y", max_y_,  1.5);

    pnh_.param<double>("min_z", min_z_, -1.0);

    pnh_.param<double>("max_z", max_z_,  0.2);



    pnh_.param<double>(

      "ransac_distance",

      ransac_distance_,

      0.02);



    sub_ = nh_.subscribe(

      cloud_topic_,

      1,

      &FloorChecker::callback,

      this);

  }



private:

  void callback(

    const sensor_msgs::PointCloud2::ConstPtr& msg)

  {

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(

      new pcl::PointCloud<pcl::PointXYZI>());



    pcl::fromROSMsg(*msg, *cloud);



    pcl::PointCloud<pcl::PointXYZI>::Ptr roi(

      new pcl::PointCloud<pcl::PointXYZI>());



    pcl::CropBox<pcl::PointXYZI> crop;

    crop.setInputCloud(cloud);



    crop.setMin(

      Eigen::Vector4f(

        min_x_, min_y_, min_z_, 1.0));



    crop.setMax(

      Eigen::Vector4f(

        max_x_, max_y_, max_z_, 1.0));



    crop.filter(*roi);



    if (roi->size() < 100)

    {

      ROS_WARN_THROTTLE(

        2.0,

        "Too few floor ROI points: %zu",

        roi->size());

      return;

    }



    pcl::SACSegmentation<pcl::PointXYZI> seg;



    seg.setOptimizeCoefficients(true);

    seg.setModelType(pcl::SACMODEL_PLANE);

    seg.setMethodType(pcl::SAC_RANSAC);

    seg.setDistanceThreshold(ransac_distance_);

    seg.setInputCloud(roi);



    pcl::PointIndices inliers;

    pcl::ModelCoefficients coeff;



    seg.segment(inliers, coeff);



    if (coeff.values.size() < 4 ||

        inliers.indices.size() < 80)

    {

      ROS_WARN_THROTTLE(

        2.0,

        "No stable floor plane.");

      return;

    }



    double a = coeff.values[0];

    double b = coeff.values[1];

    double c = coeff.values[2];



    double n = std::sqrt(a*a + b*b + c*c);



    a /= n;

    b /= n;

    c /= n;



    if (c < 0.0)

    {

      a = -a;

      b = -b;

      c = -c;

    }



    const double pitch_err =

      std::atan2(-a, c);



    const double roll_err =

      std::atan2(b, c);



    ROS_INFO_THROTTLE(

      1.0,

      "floor normal=[%.5f %.5f %.5f], "

      "roll_residual=%.3f deg, "

      "pitch_residual=%.3f deg, "

      "inliers=%zu",

      a,

      b,

      c,

      roll_err * 180.0 / M_PI,

      pitch_err * 180.0 / M_PI,

      inliers.indices.size());

  }



private:

  ros::NodeHandle nh_;

  ros::NodeHandle pnh_;

  ros::Subscriber sub_;



  std::string cloud_topic_;



  double min_x_;

  double max_x_;

  double min_y_;

  double max_y_;

  double min_z_;

  double max_z_;

  double ransac_distance_;

};



int main(int argc, char** argv)

{

  ros::init(

    argc,

    argv,

    "extrinsic_floor_checker");



  FloorChecker checker;



  ros::spin();



  return 0;

}

