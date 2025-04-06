#include "subscriber/gps_odom_subscriber.h"

#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <deque>
#include <mutex>

GpsOdometrySubscriber::GpsOdometrySubscriber(ros::NodeHandle &nh, const std::string &topic) {
        sub_ = nh.subscribe(topic, 10, &GpsOdometrySubscriber::odomCallback, this);
}

void GpsOdometrySubscriber::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    odom_buffer_.push_back(*msg);
    if (odom_buffer_.size() > max_buffer_size_) {
        odom_buffer_.pop_front();
    }
}

std::deque<nav_msgs::Odometry> GpsOdometrySubscriber::getBuffer() {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return odom_buffer_;
}

nav_msgs::Odometry GpsOdometrySubscriber::getBufferFront(){
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    auto result = odom_buffer_.front();
    odom_buffer_.pop_front();
}
