#include "perception.h"

#include "ros/ros.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>

CPerception::CPerception(ros::NodeHandle& nh, bool is_label, bool save_ipm)
    : it_(nh), is_label_(is_label), save_ipm_(save_ipm){
  // Load parameters
  std::string config_path, weight_path;
  nh.param<std::string>("config_path", config_path,
    ros::package::getPath("perception") + "/config/bisenetv1_SUPS.py");
  nh.param<std::string>("weight_path", weight_path,
    ros::package::getPath("perception") + "/models/model_final.pth");

  image_pub_ = it_.advertise("ipm_raw", 6);

	ipm_.AddCamera(CONFIG_DIR "0_intrinsic.yaml", CONFIG_DIR "0_extrinsic.yaml");
	ipm_.AddCamera(CONFIG_DIR "1_intrinsic.yaml", CONFIG_DIR "1_extrinsic.yaml");
	ipm_.AddCamera(CONFIG_DIR "2_intrinsic.yaml", CONFIG_DIR "2_extrinsic.yaml");
	ipm_.AddCamera(CONFIG_DIR "3_intrinsic.yaml", CONFIG_DIR "3_extrinsic.yaml");
    ipm_.CreateIPMToImageMap();
//    cv::namedWindow("perception", cv::WINDOW_NORMAL);

    if(save_ipm){
    	ofs_train_csv_.open(IPM_TRAIN_DIR "train.csv");
    	// Check if the file opened successfully
    	if (!ofs_train_csv_.is_open()) {
    		std::cerr << "Failed to open file!" << std::endl;
            exit(-1);
    	}
    }
}
CPerception::~CPerception(){
	if(save_ipm_){
       ofs_train_csv_.close();
	}
}

void CPerception::AddImu(const sensor_msgs::ImuConstPtr& imu){}

void CPerception::AddGps(const nav_msgs::OdometryConstPtr& gps){}

void CPerception::GetIPMImage(const sensor_msgs::CompressedImageConstPtr& image0,
              const sensor_msgs::CompressedImageConstPtr& image1,
              const sensor_msgs::CompressedImageConstPtr& image2,
              const sensor_msgs::CompressedImageConstPtr& image3){
	std::vector<cv::Mat> raw_images(4);
    raw_images[0] = cv::imdecode(image0->data, cv::IMREAD_COLOR);
    raw_images[1] = cv::imdecode(image1->data, cv::IMREAD_COLOR);
    raw_images[2] = cv::imdecode(image2->data, cv::IMREAD_COLOR);
    raw_images[3] = cv::imdecode(image3->data, cv::IMREAD_COLOR);
    auto ipm_img = ipm_.GenerateIPMImage(raw_images);
    cv::imshow("perception", ipm_img);

    if(save_ipm_){
    	ros::Time timestamp = image0->header.stamp;
    	ROS_INFO("Timestamp: sec=%d, nsec=%d", timestamp.sec, timestamp.nsec);
    	std::string file_name = std::to_string(timestamp.sec) + "." + std::to_string(timestamp.nsec) + ".png";
      if(is_label_){
        std::cout << "saving to label path: " << IPM_LABEL_DIR << std::endl;
        cv::imwrite(IPM_LABEL_DIR + file_name, ipm_img);
      }else{
        std::cout << "saving to raw path: " << IPM_RAW_DIR << std::endl;
        cv::imwrite(IPM_RAW_DIR + file_name, ipm_img);
      }
      ofs_train_csv_ << "./data/images/" + file_name << ",./data/labels/" + file_name << std::endl;
    }

    try {
      // Publish result
      std_msgs::Header header;
      header.stamp = image0->header.stamp;  // Set timestamp
      header.frame_id = "ipm_frame";    // Optional, for TF or visualization in RViz

      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", ipm_img).toImageMsg(); //mono8
      image_pub_.publish(msg);
    } catch (const std::exception& e) {
      ROS_ERROR("perception failed: %s", e.what());
    }
    cv::waitKey(1);

}
