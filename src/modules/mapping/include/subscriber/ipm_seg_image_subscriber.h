#pragma once

#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <deque>
#include <mutex>

class IPMSegImageSubscriber {
    public:
    IPMSegImageSubscriber(ros::NodeHandle &nh, const std::string &topic) ;
    void ipmSegImageCallback(const sensor_msgs::CompressedImageConstPtr &msg);

    std::deque<sensor_msgs::CompressedImage> getBuffer() ;
    sensor_msgs::CompressedImage getBufferFront();
    
    private:
        ros::Subscriber sub_;
        std::deque<sensor_msgs::CompressedImage> image_buffer_;
        std::mutex buffer_mutex_;
        size_t max_buffer_size_ = 20;  // Adjust as needed
    };
    