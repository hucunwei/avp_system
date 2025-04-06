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

int main(int argc, char **argv) {
  ros::init(argc, argv, "avp_mapping");
  ros::NodeHandle nh("~");
  bool show_img, auto_run;
  nh.param("show_img", show_img, true);
  nh.param("auto_run", auto_run, true);

  auto viewer = std::make_shared<RosViewer>(nh, show_img, auto_run);
  AvpMapping mapping;
  mapping.setViewer(viewer);

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

  // save map data
  mapping.getMap().save(path + "/avp_map.bin");
  return 0;
}
