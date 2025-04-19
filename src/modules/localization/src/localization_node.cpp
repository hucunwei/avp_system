#include "ros/ros.h"

#include "subscriber/chassis_imu_data_subscriber.h"
#include "subscriber/ipm_seg_image_subscriber.h"
#include "subscriber/gps_odom_subscriber.h"

#include "publisher/pose_publisher.h"

#include "data_pretreat/data_pretreat.h"

#include "localization.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "localization_node");
    ros::NodeHandle nh("~");

    std::string topic_ipm_seg;
    std::string topic_chassis;
    std::string topic_gps_odom;
    std::string topic_out_pose;
    std::string topic_imu;
    nh.param("topics/ipm_seg", topic_ipm_seg, std::string("/ipm_image"));
    nh.param("topics/chassis", topic_chassis, std::string("/chassis"));
    nh.param("topics/gps_odom", topic_gps_odom, std::string("/gps_odom"));
    nh.param("topics/out_pose", topic_out_pose, std::string("/localization/out_pose"));
    nh.param("topics/imu", topic_imu, std::string("/imu"));

    std::string rosbag_path;
    std::string map_path;
    nh.param("data/rosbag_path", rosbag_path, std::string(""));
    nh.param("data/map_path", map_path, std::string(""));

    //订阅底盘数据， IMU数据
    ChassisImuDataSubscriber chassis_imu_data_subscriber(nh, topic_chassis, topic_imu);
    //订阅IPM 语义分割后的图像数据
    IPMSegImageSubscriber ipm_seg_image_subscriber(nh, topic_ipm_seg);
    //订阅GNSS数据
    GpsOdometrySubscriber gps_odom_subscriber(nh, topic_gps_odom);

    //发布定位结果
    CPosePublisher pose_publisher(nh, topic_out_pose);

    std::shared_ptr<AvpLocalization> avp_localization_(new AvpLocalization(map_path));
    
    ros::Rate rate(20);
    bool ekf_inited_ = false;
    while (ros::ok())
    {
        ros::spinOnce();
        if (!ekf_inited_) 
        {
            nav_msgs::Odometry gps_odom_msg;
            bool flag = gps_odom_subscriber.getBufferFront(gps_odom_msg);
            if(flag)
            {
                //采用第一个gnss数据来初始化位姿
                TimedPose pose;
                CDatapretreat::gps_odom_convert(gps_odom_msg, pose);
               //  avp_localization_->initState(gps_odom_msg.header.stamp.toSec(), pose.t_.x() - 1., pose.t_.y() + 1.,
                //                             GetYaw(pose.R_) + 5. * kToRad);

                avp_localization_->initState(gps_odom_msg.header.stamp.toSec(), pose.t_.x() - 0.5, pose.t_.y() + 0.5,
                                            GetYaw(pose.R_) + 2.0 * kToRad);
                ekf_inited_ = true;
            }

        }
        else
        {
            auto chassis_data_buffer = chassis_imu_data_subscriber.getBuffer(true);
            for (const auto &data_elem : chassis_data_buffer) {

                WheelMeasurement wheel_meas = data_elem;
                //CDatapretreat::chassis_convert(msg, wheel_meas);
                avp_localization_->processWheelMeasurement(wheel_meas);
            }

            if(ipm_seg_image_subscriber.isBufferEmpty() == false)
            {
                sensor_msgs::CompressedImage ipm_image = ipm_seg_image_subscriber.getBufferFront();

                cv::Mat cv_image;
                CDatapretreat::ipm_seg_convert(ipm_image, cv_image);
                avp_localization_->processImage(ipm_image.header.stamp.toSec(), cv_image);

                //发布定位结果
                TimedPose pose = avp_localization_->getPose();
                pose_publisher.pubOdom(pose);
            }
        }

        rate.sleep();
    }

}