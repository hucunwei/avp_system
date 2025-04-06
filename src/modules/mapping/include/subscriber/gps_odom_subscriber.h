#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <deque>
#include <mutex>
class GpsOdometrySubscriber {
public:
    GpsOdometrySubscriber(ros::NodeHandle &nh, const std::string &topic);

    void odomCallback(const nav_msgs::OdometryConstPtr &msg) ;

    std::deque<nav_msgs::Odometry> getBuffer() ;

    nav_msgs::Odometry getBufferFront();

private:
    ros::Subscriber sub_;
    std::deque<nav_msgs::Odometry> odom_buffer_;
    std::mutex buffer_mutex_;
    size_t max_buffer_size_ = 20;
};