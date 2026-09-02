//
// Created by bruce on 2022/3/29.
//

#include <chrono>
#include <cmath>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Eigen>

#include "pclomp/ndt_omp.h"

using namespace std;

typedef pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> NDT;
typedef pcl::PointCloud<pcl::PointXYZI> Cloud;


class Config
{
public:
    string mapFrame = "map";
    // 当前 Scout 系统要求 fast_lio_localization 发布：
    //
    // map -> odom
    //
    // 不再使用原始项目默认的 map -> camera_init。
    string odomFrame = "odom";
    string baseFrame = "base_link";
    string cloudTopic = "/cloud_registered_base";
    string odomTopic = "/odom_nav";
    string localizationTopic = "/localization";
    string healthTopic = "/localization/ok";

    // map -> odom TF 向未来预发布的时间。
    // 用于避免 TEB / move_base 控制周期与 localization TF
    // 发布周期之间几十毫秒的相位差导致 future extrapolation。
    double tfPostdateSec = 0.50;

    struct
    {
        bool debug = false;
        int numThreads = 4;
        int maximumIterations = 20;
        float voxelLeafSize = 0.1;
        float resolution = 1.0;
        double transformationEpsilon = 0.01;
        double stepSize = 0.1;
        double threshShift = 2;
        double threshRot = M_PI / 12;
        double minScanRange = 1.0;
        double maxScanRange = 100;
        double maxFitnessScore = 2.0;
        double maxTranslationJump = 0.8;
        double maxRotationJump = 20.0 * M_PI / 180.0;
        double maxAlignmentAgeSec = 1.0;
    } ndt;

    explicit Config(ros::NodeHandle &nh) : _nh(nh)
    {
        // 必须读取 odom_frame。
        // 如果 launch 中没有设置，则默认使用 "odom"。
        _nh.param<string>("map_frame", mapFrame, string("map"));
        _nh.param<string>("odom_frame", odomFrame, string("odom"));
        _nh.param<string>("base_frame", baseFrame, string("base_link"));
        _nh.param<string>("cloud_topic", cloudTopic, string("/cloud_registered_base"));
        _nh.param<string>("odom_topic", odomTopic, string("/odom_nav"));
        _nh.param<string>("localization_topic", localizationTopic, string("/localization"));
        _nh.param<string>("health_topic", healthTopic, string("/localization/ok"));

        // map -> odom TF 向未来预发布时间。
        _nh.param("tf_postdate_sec", tfPostdateSec, 0.50);

        _nh.getParam("ndt/debug", ndt.debug);
        _nh.getParam("ndt/num_threads", ndt.numThreads);
        _nh.getParam("ndt/maximum_iterations", ndt.maximumIterations);
        _nh.getParam("ndt/voxel_leaf_size", ndt.voxelLeafSize);
        _nh.getParam("ndt/transformation_epsilon", ndt.transformationEpsilon);
        _nh.getParam("ndt/step_size", ndt.stepSize);
        _nh.getParam("ndt/resolution", ndt.resolution);
        _nh.getParam("ndt/thresh_shift", ndt.threshShift);
        _nh.getParam("ndt/thresh_rot", ndt.threshRot);
        _nh.getParam("ndt/min_scan_range", ndt.minScanRange);
        _nh.getParam("ndt/max_scan_range", ndt.maxScanRange);
        _nh.param("ndt/max_fitness_score", ndt.maxFitnessScore, 2.0);
        _nh.param("ndt/max_translation_jump", ndt.maxTranslationJump, 0.8);
        _nh.param(
                "ndt/max_rotation_jump",
                ndt.maxRotationJump,
                20.0 * M_PI / 180.0);
        _nh.param(
                "ndt/max_alignment_age_sec",
                ndt.maxAlignmentAgeSec,
                1.0);

        ROS_INFO("fast_lio_localization config:");
        ROS_INFO("  odom_frame      = %s", odomFrame.c_str());
        ROS_INFO("  cloud_topic     = %s", cloudTopic.c_str());
        ROS_INFO("  odom_topic      = %s", odomTopic.c_str());
        ROS_INFO("  tf_postdate_sec = %.3f", tfPostdateSec);
    }

private:
    ros::NodeHandle &_nh;
};


class Localizer
{
public:
    explicit Localizer(ros::NodeHandle &nh) :
            _nh(nh),
            _cfg(nh),
            _mapPtr(new Cloud),
            _mapFilteredPtr(new Cloud)
    {
        _localizationPub = _nh.advertise<geometry_msgs::PoseStamped>(
                _cfg.localizationTopic, 10, true);
        _scorePub = _nh.advertise<std_msgs::Float64>(
                "/localization/ndt_score", 10, true);
        _iterationsPub = _nh.advertise<std_msgs::Int32>(
                "/localization/ndt_iterations", 10, true);
        _translationJumpPub = _nh.advertise<std_msgs::Float64>(
                "/localization/translation_jump", 10, true);
        _rotationJumpPub = _nh.advertise<std_msgs::Float64>(
                "/localization/rotation_jump", 10, true);
        _lastSuccessAgePub = _nh.advertise<std_msgs::Float64>(
                "/localization/last_success_age", 10, true);

        _mapSub = _nh.subscribe(
                "/map_cloud",
                10,
                &Localizer::mapCallback,
                this
        );

        _initPoseSub = _nh.subscribe(
                "/initialpose",
                10,
                &Localizer::initPoseWithNDTCallback,
                this
        );

        _pcSubPtr =
                new message_filters::Subscriber<sensor_msgs::PointCloud2>(
                        nh,
                        _cfg.cloudTopic,
                        1
                );

        _odomSubPtr =
                new message_filters::Subscriber<nav_msgs::Odometry>(
                        nh,
                        _cfg.odomTopic,
                        1
                );

        _syncPtr =
                new message_filters::Synchronizer<syncPolicy>(
                        syncPolicy(10),
                        *_pcSubPtr,
                        *_odomSubPtr
                );

        _syncPtr->registerCallback(
                boost::bind(
                        &Localizer::syncCallback,
                        this,
                        _1,
                        _2
                )
        );

        _voxelGridFilter.setLeafSize(
                _cfg.ndt.voxelLeafSize,
                _cfg.ndt.voxelLeafSize,
                _cfg.ndt.voxelLeafSize
        );

        _ndt.setNumThreads(_cfg.ndt.numThreads);
        _ndt.setTransformationEpsilon(_cfg.ndt.transformationEpsilon);
        _ndt.setStepSize(_cfg.ndt.stepSize);
        _ndt.setResolution(_cfg.ndt.resolution);
        _ndt.setMaximumIterations(_cfg.ndt.maximumIterations);

        _odomMap.setIdentity();
    }

private:
    ros::NodeHandle &_nh;

    ros::Subscriber _mapSub;
    ros::Subscriber _initPoseSub;
    ros::Publisher _localizationPub;
    ros::Publisher _scorePub;
    ros::Publisher _iterationsPub;
    ros::Publisher _translationJumpPub;
    ros::Publisher _rotationJumpPub;
    ros::Publisher _lastSuccessAgePub;

    tf2_ros::TransformBroadcaster _br;

    message_filters::Subscriber<sensor_msgs::PointCloud2> *_pcSubPtr;
    message_filters::Subscriber<nav_msgs::Odometry> *_odomSubPtr;

    typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::PointCloud2,
            nav_msgs::Odometry
    > syncPolicy;

    message_filters::Synchronizer<syncPolicy> *_syncPtr;

    NDT _ndt;
    pcl::VoxelGrid<pcl::PointXYZI> _voxelGridFilter;

    Config _cfg;

    Cloud::Ptr _mapPtr;
    Cloud::Ptr _mapFilteredPtr;

    tf::Pose _baseOdom;
    tf::Pose _odomMap;

    sensor_msgs::PointCloud2::ConstPtr _pcPtr = nullptr;
    bool _haveValidAlignment = false;
    ros::WallTime _lastSuccessWallTime;
    bool _haveAlignmentAttempt = false;
    ros::WallTime _lastAlignmentAttemptWallTime;


    static bool finitePose(const geometry_msgs::Pose &pose)
    {
        const double qNormSquared =
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
                std::isfinite(qNormSquared) &&
                qNormSquared > 1e-12;
    }


    static bool finiteTransform(const tf::Transform &transform)
    {
        const tf::Vector3 &origin = transform.getOrigin();
        const tf::Quaternion &rotation = transform.getRotation();
        return
                std::isfinite(origin.x()) &&
                std::isfinite(origin.y()) &&
                std::isfinite(origin.z()) &&
                std::isfinite(rotation.x()) &&
                std::isfinite(rotation.y()) &&
                std::isfinite(rotation.z()) &&
                std::isfinite(rotation.w()) &&
                std::isfinite(rotation.length2()) &&
                rotation.length2() > 1e-12;
    }


    void mapCallback(
            const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        ROS_INFO("Get map");

        pcl::fromROSMsg<pcl::PointXYZI>(
                *msg,
                *_mapPtr
        );

        _ndt.setInputTarget(_mapPtr);
    }


    void initPoseCallback(
            const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
    {
        if (!finitePose(msg->pose.pose))
        {
            ROS_ERROR("Rejected non-finite initial pose");
            return;
        }

        auto &q = msg->pose.pose.orientation;
        auto &p = msg->pose.pose.position;

        tf::Pose baseMap(
                tf::Quaternion(
                        q.x,
                        q.y,
                        q.z,
                        q.w
                ),
                tf::Vector3(
                        p.x,
                        p.y,
                        p.z
                )
        );

        _odomMap =
                baseMap *
                _baseOdom.inverse();

        ROS_INFO("Initial pose set");
    }


    void initPoseWithNDTCallback(
            const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
    {
        if (!finitePose(msg->pose.pose))
        {
            ROS_ERROR("Rejected non-finite initial pose");
            return;
        }

        if (_pcPtr == nullptr)
        {
            ROS_WARN("No point cloud");
            return;
        }

        ROS_INFO("Initial pose set");

        auto &q = msg->pose.pose.orientation;
        auto &p = msg->pose.pose.position;

        tf::Pose baseMap(
                tf::Quaternion(
                        q.x,
                        q.y,
                        q.z,
                        q.w
                ),
                tf::Vector3(
                        p.x,
                        p.y,
                        p.z
                )
        );

        if (match(
                _pcPtr,
                baseMap
        ))
        {
            publishTF();
        }
    }


    void syncCallback(
            const sensor_msgs::PointCloud2::ConstPtr &pcMsg,
            const nav_msgs::Odometry::ConstPtr &odomMsg)
    {
        _pcPtr = pcMsg;

        if (!finitePose(odomMsg->pose.pose))
        {
            ROS_ERROR_THROTTLE(
                    1.0,
                    "Rejected non-finite odometry pose");
            return;
        }

        tf::poseMsgToTF(
                odomMsg->pose.pose,
                _baseOdom
        );
        tf::Quaternion normalizedOdomRotation = _baseOdom.getRotation();
        normalizedOdomRotation.normalize();
        _baseOdom.setRotation(normalizedOdomRotation);

        static tf::Pose lastNDTPose = _baseOdom;

        auto T =
                lastNDTPose.inverseTimes(
                        _baseOdom
                );

        const double shift =
                hypot(
                        T.getOrigin().x(),
                        T.getOrigin().y()
                );

        const double rotation =
                std::fabs(
                        tf::getYaw(
                                T.getRotation()
                        )
                );

        const bool periodicRefresh =
                _haveValidAlignment &&
                (!_haveAlignmentAttempt ||
                 (ros::WallTime::now() -
                  _lastAlignmentAttemptWallTime).toSec() >
                 _cfg.ndt.maxAlignmentAgeSec);

        if (shift > _cfg.ndt.threshShift ||
            rotation > _cfg.ndt.threshRot ||
            periodicRefresh)
        {
            if (match(
                    pcMsg,
                    _odomMap * _baseOdom
            ))
            {
                lastNDTPose = _baseOdom;
            }
        }

        if (_haveValidAlignment)
        {
            publishLocalization(
                    _odomMap * _baseOdom
            );
            publishLastSuccessAge();

            // 即使没有触发新的 NDT，
            // 也持续重新发布当前 map -> odom 修正值。
            publishTF();
        }
    }


    /**
     * Matching the point cloud with map to calculate `_odomMap`.
     *
     * @param pcPtr  The point cloud for matching.
     * @param baseMap The guess matrix.
     */
    bool match(
            const sensor_msgs::PointCloud2::ConstPtr &pcPtr,
            const tf::Transform &baseMap)
    {
        _lastAlignmentAttemptWallTime = ros::WallTime::now();
        _haveAlignmentAttempt = true;

        if (!finiteTransform(baseMap))
        {
            ROS_ERROR("NDT rejected: non-finite initial transform");
            return false;
        }

        static chrono::steady_clock::time_point t0;
        static chrono::steady_clock::time_point t1;

        Cloud::Ptr tmpCloudPtr(
                new Cloud
        );

        pcl::fromROSMsg(
                *pcPtr,
                *tmpCloudPtr
        );

        Cloud::Ptr filteredCloudPtr(
                new Cloud
        );

        _voxelGridFilter.setInputCloud(
                tmpCloudPtr
        );

        _voxelGridFilter.filter(
                *filteredCloudPtr
        );

        Cloud::Ptr scanCloudPtr(
                new Cloud
        );

        for (const auto &p : *filteredCloudPtr)
        {
            const auto r =
                    hypot(
                            p.x,
                            p.y
                    );

            if (r > _cfg.ndt.minScanRange &&
                r < _cfg.ndt.maxScanRange)
            {
                scanCloudPtr->push_back(p);
            }
        }

        if (scanCloudPtr->empty())
        {
            ROS_ERROR("NDT rejected: filtered scan is empty");
            return false;
        }

        _ndt.setInputSource(
                scanCloudPtr
        );

        Eigen::Affine3d baseMapMat;

        tf::poseTFToEigen(
                baseMap,
                baseMapMat
        );

        Cloud::Ptr outputCloudPtr(
                new Cloud
        );

        if (_cfg.ndt.debug)
        {
            t0 = chrono::steady_clock::now();
        }

        _ndt.align(
                *outputCloudPtr,
                baseMapMat.matrix().cast<float>()
        );

        if (_cfg.ndt.debug)
        {
            t1 = chrono::steady_clock::now();
        }

        const auto tNDT =
                _ndt.getFinalTransformation();

        const bool converged = _ndt.hasConverged();
        const double fitnessScore = _ndt.getFitnessScore();
        const int finalIterations = _ndt.getFinalNumIteration();

        std_msgs::Float64 score;
        score.data = fitnessScore;
        _scorePub.publish(score);
        std_msgs::Int32 iterations;
        iterations.data = finalIterations;
        _iterationsPub.publish(iterations);

        if (!converged ||
            !std::isfinite(fitnessScore) ||
            fitnessScore > _cfg.ndt.maxFitnessScore ||
            !tNDT.allFinite())
        {
            ROS_ERROR(
                    "NDT rejected: converged=%d score=%.6f limit=%.6f finite=%d",
                    converged,
                    fitnessScore,
                    _cfg.ndt.maxFitnessScore,
                    tNDT.allFinite());
            return false;
        }

        tf::Transform baseMapNDT;

        tf::poseEigenToTF(
                Eigen::Affine3d(
                        tNDT.cast<double>()
                ),
                baseMapNDT
        );

        if (!finiteTransform(baseMapNDT))
        {
            ROS_ERROR("NDT rejected: non-finite aligned pose");
            return false;
        }

        // 计算：
        //
        // T_map_odom =
        // T_map_base *
        // inverse(T_odom_base)
        //
        // 当前 Scout 系统中，
        // 这个 correction 最终作为：
        //
        // map -> odom
        //
        // 发布。
        const tf::Transform previousOdomMap = _odomMap;
        const tf::Transform candidateOdomMap =
                baseMapNDT *
                _baseOdom.inverse();

        if (!finiteTransform(candidateOdomMap))
        {
            ROS_ERROR("NDT rejected: non-finite map-to-odom correction");
            return false;
        }

        const tf::Transform correctionJump =
                previousOdomMap.inverseTimes(candidateOdomMap);
        const double translationJump = correctionJump.getOrigin().length();
        const double rotationJump =
                std::fabs(tf::getYaw(correctionJump.getRotation()));

        if (!std::isfinite(translationJump) ||
            !std::isfinite(rotationJump))
        {
            ROS_ERROR("NDT rejected: non-finite correction jump");
            return false;
        }

        if (_haveValidAlignment &&
            (translationJump > _cfg.ndt.maxTranslationJump ||
             rotationJump > _cfg.ndt.maxRotationJump))
        {
            ROS_ERROR(
                    "NDT rejected: correction jump %.3f m / %.3f rad",
                    translationJump,
                    rotationJump);
            return false;
        }

        std_msgs::Float64 translation_jump;
        translation_jump.data = _haveValidAlignment ? translationJump : 0.0;
        _translationJumpPub.publish(translation_jump);
        std_msgs::Float64 rotation_jump;
        rotation_jump.data = _haveValidAlignment ? rotationJump : 0.0;
        _rotationJumpPub.publish(rotation_jump);

        _odomMap = candidateOdomMap;
        _haveValidAlignment = true;
        _lastSuccessWallTime = ros::WallTime::now();
        publishLastSuccessAge();
        publishLocalization(baseMapNDT);

        if (_cfg.ndt.debug)
        {
            ROS_INFO(
                    "NDT: %ldms",
                    chrono::duration_cast<chrono::milliseconds>(
                            t1 - t0
                    ).count()
            );
        }

        ROS_INFO("NDT Relocated");
        return true;
    }


    void publishLocalization(
            const tf::Transform &baseMap)
    {
        if (!finiteTransform(baseMap))
        {
            ROS_ERROR_THROTTLE(
                    1.0,
                    "Suppressed non-finite localization pose");
            return;
        }

        geometry_msgs::PoseStamped localization;
        localization.header.stamp = ros::Time::now();
        localization.header.frame_id = _cfg.mapFrame;
        tf::poseTFToMsg(baseMap, localization.pose);
        _localizationPub.publish(localization);
    }


    void publishLastSuccessAge()
    {
        if (!_haveValidAlignment)
        {
            return;
        }

        std_msgs::Float64 age;
        age.data =
                (ros::WallTime::now() -
                 _lastSuccessWallTime).toSec();
        _lastSuccessAgePub.publish(age);
    }


    void publishTF()
    {
        if (!_haveValidAlignment ||
            !finiteTransform(_odomMap))
        {
            return;
        }

        geometry_msgs::TransformStamped tfMsg;

        // 关键修改：
        //
        // map -> odom 按 tf_postdate_sec 向未来预发布，
        // 避免 TEB 查询当前时刻 TF 时，
        // 最新 map -> odom 尚落后几十毫秒而出现：
        //
        // Lookup would require extrapolation into the future
        //
        tfMsg.header.stamp =
                ros::Time::now() +
                ros::Duration(
                        _cfg.tfPostdateSec
                );

        tfMsg.header.frame_id = _cfg.mapFrame;

        // 必须是 odom。
        tfMsg.child_frame_id =
                _cfg.odomFrame;

        tfMsg.transform.translation.x =
                _odomMap.getOrigin().x();

        tfMsg.transform.translation.y =
                _odomMap.getOrigin().y();

        tfMsg.transform.translation.z =
                _odomMap.getOrigin().z();

        tfMsg.transform.rotation.x =
                _odomMap.getRotation().x();

        tfMsg.transform.rotation.y =
                _odomMap.getRotation().y();

        tfMsg.transform.rotation.z =
                _odomMap.getRotation().z();

        tfMsg.transform.rotation.w =
                _odomMap.getRotation().w();

        _br.sendTransform(
                tfMsg
        );
    }
};


int main(
        int argc,
        char **argv)
{
    ros::init(
            argc,
            argv,
            "fast_lio_localization"
    );

    ros::NodeHandle nh("~");

    Localizer localizer(
            nh
    );

    ros::spin();

    return 0;
}
