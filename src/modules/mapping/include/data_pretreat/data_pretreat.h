/**
 * 数据预处理
 */
#pragma once

#include <nav_msgs/Odometry.h>

#include <sensor_msgs/CompressedImage.h>
#include "pose_interpolation.h"

#include <opencv2/opencv.hpp>

class CDatapretreat
{
private:
    /* data */
public:
    CDatapretreat(/* args */);
    ~CDatapretreat();

    //ipm seg 消息类型 转换
    static bool ipm_seg_convert(const sensor_msgs::CompressedImage &ipm_seg_msg, cv::Mat& image);

    //chassis消息类型转换
    static bool gps_odom_convert(const nav_msgs::Odometry &gps_odom_msg, TimedPose& pose);
};

