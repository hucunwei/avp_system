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


bool save_ipm = false;
std::string seg = "";

int main(int argc, char **argv) {
  ros::init(argc, argv, "perception_node");
  ros::NodeHandle nh("~");


  bool is_label = false;

  nh.param("is_label", is_label, false);
  nh.param("save_ipm", save_ipm, false);

  // bool enable_infer_node;
  // nh.param("enable_infer_node", enable_infer_node, true);

  if (is_label) {
    seg = "_seg";
  }
  std::cout << "is_label: " << is_label << std::endl;
  std::cout << "save_ipm: " << save_ipm << std::endl;
  // std::cout << "enable_infer_node: " << enable_infer_node << std::endl;

  std::string front_topic;
  std::string back_topic;
  std::string left_topic;
  std::string right_topic;
  nh.param("topics/camera_front", front_topic, std::string("/camera/front"));
  nh.param("topics/camera_back", back_topic, std::string("/camera/back"));
  nh.param("topics/camera_left", left_topic, std::string("/camera/left"));
  nh.param("topics/camera_right", right_topic, std::string("/camera/right"));

  front_topic = front_topic + seg;
  back_topic = back_topic + seg;
  left_topic = left_topic + seg;
  right_topic = right_topic + seg;

  std::cout << "camera_front topic: " << front_topic << ","
            << "camera_back topic: " << back_topic << ","
            << "camera_left topic: " << left_topic << ","
            << "camera_right topic: " << right_topic << std::endl;

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
