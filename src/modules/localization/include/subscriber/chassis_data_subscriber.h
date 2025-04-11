#pragma once

#include <ros/ros.h>
#include <common_msgs/CanBusData.h>
#include <deque>
#include <mutex>

class ChassisDataSubscriber {
    public:
    ChassisDataSubscriber(ros::NodeHandle &nh, const std::string &topic) ;
    void chassisDataCallback(const common_msgs::CanBusDataConstPtr &msg);

    std::deque<common_msgs::CanBusData> getBuffer(bool clear_buffer  = false ) ;
    common_msgs::CanBusData getBufferFront();

    bool isBufferEmpty() const{
        return chassis_data_buffer_.empty();
    }
    
    private:
        ros::Subscriber sub_;
        std::deque<common_msgs::CanBusData> chassis_data_buffer_;
        std::mutex buffer_mutex_;
        size_t max_buffer_size_ = 20;  // Adjust as needed
    };
    