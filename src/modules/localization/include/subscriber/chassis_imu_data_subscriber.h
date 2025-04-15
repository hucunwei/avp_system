#pragma once

#include <ros/ros.h>
#include <deque>
#include <mutex>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/Imu.h>
#include <common_msgs/CanBusData.h>
#include "data_struct.h"
// #include <geometry_msgs/Vector3Stamped.h>

typedef message_filters::sync_policies::ExactTime<sensor_msgs::Imu, common_msgs::CanBusData> sync_pol;

class ChassisImuDataSubscriber {
    public:
    ChassisImuDataSubscriber(ros::NodeHandle &nh, const std::string &chassis_topic, const std::string& imu_topic) ;
    void chassisDataCallback(const common_msgs::CanBusDataConstPtr &msg);

    void addImuSpeed(const sensor_msgs::ImuConstPtr &imu, const common_msgs::CanBusDataConstPtr &chassis_data);

    std::deque<WheelMeasurement> getBuffer(bool clear_buffer  = false ) ;
    WheelMeasurement getBufferFront();

    bool isBufferEmpty() const{
        return chassis_data_buffer_.empty();
    }
    
    private:
        // ros::Subscriber sub_;
        std::deque<WheelMeasurement> chassis_data_buffer_;
        std::mutex buffer_mutex_;
        size_t max_buffer_size_ = 20;  // Adjust as needed

        message_filters::Subscriber<sensor_msgs::Imu> sub_imu_;
        message_filters::Subscriber<common_msgs::CanBusData> sub_chassis_;
        message_filters::Synchronizer<sync_pol> sync;
    };
    