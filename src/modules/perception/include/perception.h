#pragma once

#include "ipm.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Imu.h>

class CPerception {
public:
    CPerception(bool save_ipm);
    ~CPerception();

    void AddImu(const sensor_msgs::ImuConstPtr& imu);

    void AddGps(const nav_msgs::OdometryConstPtr& gps);

  void AddImage(const sensor_msgs::CompressedImageConstPtr& image0,
    const sensor_msgs::CompressedImageConstPtr& image1,
    const sensor_msgs::CompressedImageConstPtr& image2,
    const sensor_msgs::CompressedImageConstPtr& image3,
    const sensor_msgs::CompressedImageConstPtr& image4,
     const sensor_msgs::CompressedImageConstPtr& image5,
     const sensor_msgs::CompressedImageConstPtr& image6,
     const sensor_msgs::CompressedImageConstPtr& image7);
private:
    IPM ipm_;
    bool save_ipm_ = false;
  std::ofstream ofs_train_csv_;
};
