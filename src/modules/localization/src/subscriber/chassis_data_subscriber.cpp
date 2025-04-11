#include "subscriber/chassis_data_subscriber.h"


ChassisDataSubscriber::ChassisDataSubscriber(ros::NodeHandle &nh, const std::string &topic) {
    sub_ = nh.subscribe(topic, 10, &ChassisDataSubscriber::chassisDataCallback, this);
}

void ChassisDataSubscriber::chassisDataCallback(const common_msgs::CanBusDataConstPtr &msg) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    chassis_data_buffer_.push_back(*msg);
    // Optional: Limit buffer size
    if (chassis_data_buffer_.size() > max_buffer_size_) {
        chassis_data_buffer_.pop_front();
    }
}

std::deque<common_msgs::CanBusData> ChassisDataSubscriber::getBuffer(bool clear_buffer /* = false */) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if(!clear_buffer)
    {
        return chassis_data_buffer_;
    }
    else
    {
        std::deque<common_msgs::CanBusData> result;
        result.swap(chassis_data_buffer_);
        return result;
    }
}

common_msgs::CanBusData ChassisDataSubscriber::getBufferFront(){
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    auto result = chassis_data_buffer_.front();
    chassis_data_buffer_.pop_front();
    return result;
}
    

    