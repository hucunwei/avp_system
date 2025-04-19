#include "subscriber/gps_odom_subscriber.h"
#include "data_struct.h"

GpsOdometrySubscriber::GpsOdometrySubscriber(ros::NodeHandle &nh, const std::string &topic) {
        sub_ = nh.subscribe(topic, 10, &GpsOdometrySubscriber::odomCallback, this);
}

void GpsOdometrySubscriber::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    odom_buffer_.push_back(*msg);

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

    // static int ds = 0;
    // if(ds++ % 50 == 0)
    // {
    //     std::cout << std::fixed << std::setprecision(2) << "gps odom:" << pose.time_ << ", pose:" << position.transpose() 
    //                 << ", yaw: " <<  GetYaw(q) * 180.0 / 3.1416
    //                 << std::endl;
    // }
    

    // odom_buffer_.push_back(msg);

    if (odom_buffer_.size() > max_buffer_size_) {
        odom_buffer_.pop_front();
    }
}

std::deque<nav_msgs::Odometry> GpsOdometrySubscriber::getBuffer(bool clear_buffer/* = false */) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if(!clear_buffer)
    {
        return odom_buffer_;
    }
    else
    {
        std::deque<nav_msgs::Odometry> result;
        result.swap(odom_buffer_);
        return result;
    }
}

bool GpsOdometrySubscriber::getBufferFront(nav_msgs::Odometry& gps_odom_msg){
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if(!odom_buffer_.empty())
    {
        gps_odom_msg = odom_buffer_.front();
        odom_buffer_.pop_front();
        return true;
    }
    else
    {   
        return false;
    }    
}
