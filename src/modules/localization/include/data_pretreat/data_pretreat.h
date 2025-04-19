/**
 * 数据预处理
 */
#pragma once

#include <nav_msgs/Odometry.h>

#include <common_msgs/CanBusData.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/opencv.hpp>
#include "data_struct.h"

class CDatapretreat
{
private:
    /* data */
public:
    CDatapretreat(/* args */);
    ~CDatapretreat();

    static bool ipm_seg_convert(const sensor_msgs::CompressedImage &ipm_seg, cv::Mat& image);

    static bool chassis_convert(const common_msgs::CanBusData &chassis_data, WheelMeasurement& wheel_meas);

    static bool gps_odom_convert(const nav_msgs::Odometry &gps_odom_msg, TimedPose& pose);
};

