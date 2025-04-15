#include "subscriber/chassis_imu_data_subscriber.h"


ChassisImuDataSubscriber::ChassisImuDataSubscriber(ros::NodeHandle &nh, 
    const std::string &chassis_topic, 
    const std::string& imu_topic) : sub_chassis_(nh, chassis_topic, 100), 
    sub_imu_(nh, imu_topic, 100), sync(sync_pol(100), sub_imu_, sub_chassis_)
{
    sync.registerCallback(boost::bind(&ChassisImuDataSubscriber::addImuSpeed, this, _1, _2));
}

// void ChassisImuDataSubscriber::chassisDataCallback(const common_msgs::CanBusDataConstPtr &msg) {
//     std::lock_guard<std::mutex> lock(buffer_mutex_);
//     chassis_data_buffer_.push_back(*msg);
//     // Optional: Limit buffer size
//     if (chassis_data_buffer_.size() > max_buffer_size_) {
//         chassis_data_buffer_.pop_front();
//     }
// }

void ChassisImuDataSubscriber::addImuSpeed(const sensor_msgs::ImuConstPtr &imu,
    const common_msgs::CanBusDataConstPtr &chassis_data) 
{
  
    WheelMeasurement wheel_measurement;
    wheel_measurement.time_ = imu->header.stamp.toSec();
    wheel_measurement.velocity_ = chassis_data->linear_velocities.z;  //z轴超前
    wheel_measurement.yaw_rate_ = imu->angular_velocity.z;
    //avp_localization_->processWheelMeasurement(wheel_measurement);

    std::cout << "IMU & Speed data received, time:" << wheel_measurement.time_ << std::endl;

    chassis_data_buffer_.push_back(wheel_measurement);
    if (chassis_data_buffer_.size() > max_buffer_size_) {
        chassis_data_buffer_.pop_front();
    }

}

std::deque<WheelMeasurement> ChassisImuDataSubscriber::getBuffer(bool clear_buffer /* = false */) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if(!clear_buffer)
    {
        return chassis_data_buffer_;
    }
    else
    {
        std::deque<WheelMeasurement> result;
        result.swap(chassis_data_buffer_);
        return result;
    }
}

WheelMeasurement ChassisImuDataSubscriber::getBufferFront(){
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    auto result = chassis_data_buffer_.front();
    chassis_data_buffer_.pop_front();
    return result;
}
    

    