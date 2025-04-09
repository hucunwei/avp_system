#include "perception.h"

#include <iostream>
#include <string>
#include <geometry_msgs/Vector3Stamped.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <boost/bind.hpp>

#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>


bool save_ipm = true;
std::string seg = "";

int main(int argc, char **argv) {
  ros::init(argc, argv, "perception_node");
  ros::NodeHandle nh("~");


  bool is_label = false;
  if (is_label) {
    seg = "_seg";
  }

  std::string front_topic("/camera/front" + seg);
  std::string back_topic("/camera/back" + seg);
  std::string left_topic("/camera/left" + seg);
  std::string right_topic("/camera/right" + seg);

  message_filters::Subscriber<sensor_msgs::CompressedImage> front_sub(nh, front_topic, 100);
  message_filters::Subscriber<sensor_msgs::CompressedImage> back_sub(nh, back_topic, 100);
  message_filters::Subscriber<sensor_msgs::CompressedImage> left_sub(nh, left_topic, 100);
  message_filters::Subscriber<sensor_msgs::CompressedImage> right_sub(nh, right_topic, 100);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage,
    sensor_msgs::CompressedImage, sensor_msgs::CompressedImage> sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(10), front_sub, back_sub, left_sub, right_sub);

  CPerception perception(nh, is_label, save_ipm);
  sync.registerCallback(boost::bind(&CPerception::GetIPMImage, &perception, _1, _2, _3, _4));

  // ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(
  //   "/imu", 100, boost::bind(&CPerception::AddImu, &perception, _1));
  // ros::Subscriber gps_sub = nh.subscribe<nav_msgs::Odometry>(
  //   "/gps_odom", 100, boost::bind(&CPerception::AddGps, &perception, _1));

  ros::spin();
  return 0;
}
