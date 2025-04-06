#include "subscriber/gps_odom_subscriber.h"

GpsOdometrySubscriber::GpsOdometrySubscriber(ros::NodeHandle &nh, const std::string &topic) {
        sub_ = nh.subscribe(topic, 10, &GpsOdometrySubscriber::odomCallback, this);
}

void GpsOdometrySubscriber::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    //odom_buffer_.push_back(*msg);

    TimedPose pose;
    Eigen::Vector3d &position = pose.t_;
    Eigen::Quaterniond &q = pose.R_;
    pose.time_ = msg->header.stamp.toSec();;
    position(0) = msg->pose.pose.position.x;
    position(1) = msg->pose.pose.position.y;
    position(2) = msg->pose.pose.position.z;
    q.x() = msg->pose.pose.orientation.x;
    q.y() = msg->pose.pose.orientation.y;
    q.z() = msg->pose.pose.orientation.z;
    q.w() = msg->pose.pose.orientation.w;
   
    odom_buffer_.push_back(pose);

    if (odom_buffer_.size() > max_buffer_size_) {
        odom_buffer_.pop_front();
    }
}

std::deque<TimedPose> GpsOdometrySubscriber::getBuffer(bool clear_buffer/* = false */) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if(!clear_buffer)
    {
        return odom_buffer_;
    }
    else
    {
        std::deque<TimedPose> result;
        result.swap(odom_buffer_);
        return result;
    }
}

TimedPose GpsOdometrySubscriber::getBufferFront(){
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    auto result = odom_buffer_.front();
    odom_buffer_.pop_front();
    return result;
}
