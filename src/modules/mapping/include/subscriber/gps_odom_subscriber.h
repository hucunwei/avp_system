#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <deque>
#include <mutex>

#include "pose_interpolation.h"

class GpsOdometrySubscriber {
public:
    GpsOdometrySubscriber(ros::NodeHandle &nh, const std::string &topic);

    void odomCallback(const nav_msgs::OdometryConstPtr &msg) ;

    std::deque<TimedPose> getBuffer(bool clear_buffer = false) ;

    TimedPose getBufferFront();

private:
    ros::Subscriber sub_;
    std::deque<TimedPose> odom_buffer_;
    std::mutex buffer_mutex_;
    size_t max_buffer_size_ = 20;
};