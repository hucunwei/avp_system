#pragma once

#include "ipm.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Imu.h>

class CPerception {
public:
    CPerception(bool is_label, bool save_ipm);
    ~CPerception(){};

    void AddImu(const sensor_msgs::ImuConstPtr& imu);

    void AddGps(const nav_msgs::OdometryConstPtr& gps);

    void AddImage(const sensor_msgs::CompressedImageConstPtr& image0,
                  const sensor_msgs::CompressedImageConstPtr& image1,
                  const sensor_msgs::CompressedImageConstPtr& image2,
                  const sensor_msgs::CompressedImageConstPtr& image3);
private:
    IPM ipm_;
    bool is_label_;
    bool save_ipm_ = false;
};
