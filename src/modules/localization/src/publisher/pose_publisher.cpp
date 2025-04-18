#include "publisher/pose_publisher.h"
#include <nav_msgs/Odometry.h>

CPosePublisher::CPosePublisher(ros::NodeHandle &nh, const std::string &topic) 
{
    
    pub_ = nh.advertise<nav_msgs::Odometry>(topic, 50);
}

CPosePublisher::~CPosePublisher()
{
}

bool CPosePublisher::pubOdom(const TimedPose& pose)
{
    // 首先发布tf变换
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time(pose.time_);
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    
    odom_trans.transform.translation.x = pose.t_.x();
    odom_trans.transform.translation.y = pose.t_.y();
    odom_trans.transform.translation.z = pose.t_.z();
    odom_trans.transform.rotation.x = pose.R_.x();
    odom_trans.transform.rotation.y = pose.R_.y();
    odom_trans.transform.rotation.z = pose.R_.z();
    odom_trans.transform.rotation.w = pose.R_.w();
    // 发送变换
    odom_broadcaster.sendTransform(odom_trans);
    
    // 然后发布Odometry消息
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time(pose.time_);
    odom.header.frame_id = "odom";
    
    // 设置位姿
    odom.pose.pose.position.x = pose.t_.x();
    odom.pose.pose.position.y = pose.t_.y();
    odom.pose.pose.position.z = pose.t_.z();
    odom.pose.pose.orientation.x = pose.R_.x();
    odom.pose.pose.orientation.y = pose.R_.y();
    odom.pose.pose.orientation.z = pose.R_.z();
    odom.pose.pose.orientation.w = pose.R_.w();

    // std::cout << std::fixed << std::setprecision(4) << odom.header.stamp << ", pub odom:" << "pose:" << pose.t_.transpose() 
    // << ", yaw: " <<  GetYaw(pose.R_) * 180.0 / 3.1416
    // << std::endl;
    
    // 设置速度
    // odom.child_frame_id = "base_link";
    // odom.twist.twist.linear.x = vx;
    // odom.twist.twist.linear.y = vy;
    // odom.twist.twist.angular.z = vth;
    
    // 发布消息
    pub_.publish(odom);

    return true;
}