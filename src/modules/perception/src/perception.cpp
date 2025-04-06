#include "perception.h"

#include "ros/ros.h"
#include <iostream>
#include <opencv2/opencv.hpp>

CPerception::CPerception(bool is_label, bool save_ipm) : is_label_(is_label), save_ipm_(save_ipm){

	ipm_.AddCamera(CONFIG_DIR "0_intrinsic.yaml", CONFIG_DIR "0_extrinsic.yaml");
	ipm_.AddCamera(CONFIG_DIR "1_intrinsic.yaml", CONFIG_DIR "1_extrinsic.yaml");
	ipm_.AddCamera(CONFIG_DIR "2_intrinsic.yaml", CONFIG_DIR "2_extrinsic.yaml");
	ipm_.AddCamera(CONFIG_DIR "3_intrinsic.yaml", CONFIG_DIR "3_extrinsic.yaml");
    ipm_.CreateIPMToImageMap();
    cv::namedWindow("perception", cv::WINDOW_NORMAL);
}

void CPerception::AddImu(const sensor_msgs::ImuConstPtr& imu){}

void CPerception::AddGps(const nav_msgs::OdometryConstPtr& gps){}

void CPerception::AddImage(const sensor_msgs::CompressedImageConstPtr& image0,
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
    }
    cv::waitKey(1);
}
