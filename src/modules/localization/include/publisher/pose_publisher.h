#pragma once

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "data_struct.h"

class CPosePublisher
{
private:
    /* data */
public:
    CPosePublisher(ros::NodeHandle &nh, const std::string &topic);
    ~CPosePublisher();

    bool pubOdom(const TimedPose& pose);

private:
    ros::Publisher pub_;
    // 用于发布tf变换
    tf::TransformBroadcaster odom_broadcaster;
};
