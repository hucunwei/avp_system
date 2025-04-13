#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include "avp_mapping.h"
#include "ros_viewer/mapping_ros_viewer.h"

#include "subscriber/ipm_seg_image_subscriber.h"
#include "subscriber/gps_odom_subscriber.h"

#include "data_pretreat/data_pretreat.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "avp_mapping");
  ros::NodeHandle nh("~");
  bool show_img, auto_run;
  nh.param("show_img", show_img, true);
  nh.param("auto_run", auto_run, true);

  auto viewer = std::make_shared<RosViewer>(nh, show_img, auto_run);
  AvpMapping mapping;
  mapping.setViewer(viewer);

  std::string map_data_out_path;
  nh.param("map_data_out_path", map_data_out_path, std::string("~")); 
  std::cout << map_data_out_path << std::endl;

#if 0
  std::string ipm_data_path;
  nh.param("ipm_data_path", ipm_data_path, std::string("~/ipm_data/"));
  std::cout << ipm_data_path << std::endl;

  std::string path(ipm_data_path); // default dataset path
  if (argc > 1) {
    path = std::string(argv[1]);  // user specified dataset path
  }

  // load pose data
  std::string pose_file(path + "/poses.bin");
  std::ifstream pose_ifs(pose_file, std::ios::binary);
  if (!pose_ifs) {
    std::cerr << "Unable to open pose file: " << pose_file << std::endl;
    return 0;
  }
  std::vector<TimedPose> poses;
  io::loadVector(pose_ifs, poses);
  for (const auto &pose : poses) {
    mapping.processPose(pose);
  }
  std::cout << "poses num: " << poses.size() << std::endl;
  pose_ifs.close();

  // load and process images
  std::ifstream ipm_imgs_ifs(path + "/ipm_imgs.txt");
  if (!ipm_imgs_ifs) {
    std::cerr << "Unable to open ipm img file: " << path + "/ipm_imgs.txt" << std::endl;
    return 0;
  }
  double time;
  std::string line, img_name;
  while (ros::ok() && std::getline(ipm_imgs_ifs, line)) {
    std::stringstream ss(line);
    ss >> time >> img_name;
    std::cout << std::to_string(time) << " :  " << img_name << std::endl;
    // process image
    mapping.processImage(time, cv::imread(path + "/" + img_name));
  }
#else

  std::string ipm_seg_topic;
  nh.param("ipm_seg", ipm_seg_topic, std::string("/ipm_seg/compressed"));
  std::string truth_odom_topic;
  nh.param("truth_odom", truth_odom_topic, std::string("/gps_odom")); 

  std::cout << "ipm_seg_topic: " << ipm_seg_topic << ", truth_odom_topic: " << truth_odom_topic << std::endl;

  GpsOdometrySubscriber gps_odom_subscriber(nh, truth_odom_topic);

  IPMSegImageSubscriber ipm_seg_image_subscriber(nh, ipm_seg_topic);

  ros::Rate rate(20);
  while (ros::ok())
  {
    ros::spinOnce();

    auto gps_odom_buffer = gps_odom_subscriber.getBuffer(true);
    for (const auto &msg : gps_odom_buffer) {

      TimedPose pose;
      CDatapretreat::gps_odom_convert(msg, pose);
      mapping.processPose(pose);
    }

    if(ipm_seg_image_subscriber.isBufferEmpty() == false)
    {
      sensor_msgs::CompressedImage ipm_image = ipm_seg_image_subscriber.getBufferFront();

      cv::Mat cv_image;
      CDatapretreat::ipm_seg_convert(ipm_image, cv_image);

      mapping.processImage(ipm_image.header.stamp.toSec(), cv_image);
    }

    rate.sleep();
  }

#endif
  // save map data for areas A + B
  //ToDo(Hu): need to save the first pose into this bin according to hw requirement
  mapping.getMap().save(map_data_out_path + "avp_map_sim.bin");
  return 0;
}
