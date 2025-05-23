#include "data_pretreat/data_pretreat.h"
#include "data_struct.h"
#include <opencv2/opencv.hpp>

CDatapretreat::CDatapretreat(/* args */)
{
}

CDatapretreat::~CDatapretreat()
{
}

bool CDatapretreat::ipm_seg_convert(const sensor_msgs::CompressedImage &ipm_seg, cv::Mat& image)
{
    image = cv::imdecode(cv::Mat(ipm_seg.data), cv::IMREAD_COLOR);
    return !image.empty();
}

bool CDatapretreat::chassis_convert(const common_msgs::CanBusData &chassis_data, WheelMeasurement& wheel_meas)
{
    // wheel_meas.time_ = imu->header.stamp.toSec();
    // wheel_meas.velocity_ = speed->vector.y;
    // wheel_meas.yaw_rate_ = imu->angular_velocity.z;

    return true;
}

bool CDatapretreat::gps_odom_convert(const nav_msgs::Odometry &gps_odom_msg, TimedPose& pose)
{
    // TimedPose pose;
    Eigen::Vector3d &position = pose.t_;
    Eigen::Quaterniond &q = pose.R_;
    pose.time_ = gps_odom_msg.header.stamp.toSec();;
    position(0) = gps_odom_msg.pose.pose.position.x;
    position(1) = gps_odom_msg.pose.pose.position.y;
    position(2) = gps_odom_msg.pose.pose.position.z;
    q.x() = gps_odom_msg.pose.pose.orientation.x;
    q.y() = gps_odom_msg.pose.pose.orientation.y;
    q.z() = gps_odom_msg.pose.pose.orientation.z;
    q.w() = gps_odom_msg.pose.pose.orientation.w;

    return true;
}