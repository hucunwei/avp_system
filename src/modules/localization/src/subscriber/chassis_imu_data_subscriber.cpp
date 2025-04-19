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

    //wheel_measurement.velocity_ = chassis_data->linear_velocities.z;  //z轴朝前
    Eigen::Quaterniond q;
    q.x() = chassis_data->orientation.x;
    q.y() = chassis_data->orientation.y;
    q.z() = chassis_data->orientation.z;
    q.w() = chassis_data->orientation.w;
    // 归一化四元数
    q.normalize();

    //左手坐标系 东天北 转 右手坐标系 东北天
    Eigen::Vector3d vel_world{chassis_data->linear_velocities.x, chassis_data->linear_velocities.z, chassis_data->linear_velocities.y};
    Eigen::Vector3d vel_body = q.inverse() * vel_world; //速度从世界坐标系转到车体坐标系（右前上）

    wheel_measurement.velocity_ = vel_body(1); 


    wheel_measurement.yaw_rate_ = imu->angular_velocity.z;
    //avp_localization_->processWheelMeasurement(wheel_measurement);

    // static int ds = 0;
    // if (ds++ % 5 == 0){
    //     std::cout <<std::fixed << std::setprecision(2) << "IMU & Speed data received, time:" 
    //     << wheel_measurement.time_  << ","
    //     << "vel_body:" << vel_body(0) << "," << vel_body(1) << "," << vel_body(2) << ","
    //     << "vel_chassis:" << vel_world(0) << "," << vel_world(1) << "," << vel_world(2)
    //     << ", q:" << chassis_data->orientation.x << "," << chassis_data->orientation.y << "," << chassis_data->orientation.z << "," << chassis_data->orientation.w
    //     << std::endl;
    // }

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
    

    