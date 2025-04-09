#include "perception.h"

#include "ros/ros.h"
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

int main(int argc, char **argv) {
  ros::init(argc, argv, "perception_node");
  ros::NodeHandle nh("~");

  std::string front_topic("/camera/front");
  std::string back_topic("/camera/back");
  std::string left_topic("/camera/left");
  std::string right_topic("/camera/right");

  std::string front_topic_seg("/camera/front_seg");
  std::string back_topic_seg("/camera/back_seg");
  std::string left_topic_seg("/camera/left_seg");
  std::string right_topic_seg("/camera/right_seg");

  message_filters::Subscriber<sensor_msgs::CompressedImage> front_sub(nh, front_topic, 100);
  message_filters::Subscriber<sensor_msgs::CompressedImage> back_sub(nh, back_topic, 100);
  message_filters::Subscriber<sensor_msgs::CompressedImage> left_sub(nh, left_topic, 100);
  message_filters::Subscriber<sensor_msgs::CompressedImage> right_sub(nh, right_topic, 100);

  message_filters::Subscriber<sensor_msgs::CompressedImage> front_sub_seg(nh, front_topic_seg, 100);
  message_filters::Subscriber<sensor_msgs::CompressedImage> back_sub_seg(nh, back_topic_seg, 100);
  message_filters::Subscriber<sensor_msgs::CompressedImage> left_sub_seg(nh, left_topic_seg, 100);
  message_filters::Subscriber<sensor_msgs::CompressedImage> right_sub_seg(nh, right_topic_seg, 100);


  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage,
  sensor_msgs::CompressedImage, sensor_msgs::CompressedImage, sensor_msgs::CompressedImage, sensor_msgs::CompressedImage,
  sensor_msgs::CompressedImage, sensor_msgs::CompressedImage> sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(10), front_sub, back_sub, left_sub, right_sub, front_sub_seg, back_sub_seg, left_sub_seg, right_sub_seg);

  CPerception perception(save_ipm);
  sync.registerCallback(boost::bind(&CPerception::AddImage, &perception, _1, _2, _3, _4, _5, _6, _7, _8));
  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(
    "/imu", 100, boost::bind(&CPerception::AddImu, &perception, _1));
  ros::Subscriber gps_sub = nh.subscribe<nav_msgs::Odometry>(
    "/gps_odom", 100, boost::bind(&CPerception::AddGps, &perception, _1));

  ros::spin();
  return 0;
}
