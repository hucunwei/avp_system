#pragma once

#include "ipm.h"

#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Imu.h>
#include <image_transport/image_transport.h>

class CPerception {
public:
    CPerception(ros::NodeHandle& nh, bool is_label=false, bool save_ipm=false);
    ~CPerception();

    void AddImu(const sensor_msgs::ImuConstPtr& imu);

    void AddGps(const nav_msgs::OdometryConstPtr& gps);

    void GetIPMImage(const sensor_msgs::CompressedImageConstPtr& image0,
                  const sensor_msgs::CompressedImageConstPtr& image1,
                  const sensor_msgs::CompressedImageConstPtr& image2,
                  const sensor_msgs::CompressedImageConstPtr& image3);
private:
    IPM ipm_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    bool is_label_;
    bool save_ipm_ = false;
    std::ofstream ofs_train_csv_;
};
