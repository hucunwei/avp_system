#include "perception.h"

#include "ros/ros.h"
#include <iostream>
#include <opencv2/opencv.hpp>

CPerception::CPerception(bool save_ipm) : save_ipm_(save_ipm){

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

void CPerception::AddImage(const sensor_msgs::CompressedImageConstPtr& image0,
              const sensor_msgs::CompressedImageConstPtr& image1,
              const sensor_msgs::CompressedImageConstPtr& image2,
              const sensor_msgs::CompressedImageConstPtr& image3,
              const sensor_msgs::CompressedImageConstPtr& image4,
 const sensor_msgs::CompressedImageConstPtr& image5,
 const sensor_msgs::CompressedImageConstPtr& image6,
 const sensor_msgs::CompressedImageConstPtr& image7){
	std::vector<cv::Mat> raw_images(4);
    raw_images[0] = cv::imdecode(image0->data, cv::IMREAD_COLOR);
    raw_images[1] = cv::imdecode(image1->data, cv::IMREAD_COLOR);
    raw_images[2] = cv::imdecode(image2->data, cv::IMREAD_COLOR);
    raw_images[3] = cv::imdecode(image3->data, cv::IMREAD_COLOR);
    auto ipm_img = ipm_.GenerateIPMImage(raw_images);
//    cv::imshow("perception", ipm_img);

	std::vector<cv::Mat> label_images(4);
	label_images[0] = cv::imdecode(image4->data, cv::IMREAD_COLOR);
	label_images[1] = cv::imdecode(image5->data, cv::IMREAD_COLOR);
	label_images[2] = cv::imdecode(image6->data, cv::IMREAD_COLOR);
	label_images[3] = cv::imdecode(image7->data, cv::IMREAD_COLOR);
	auto ipm_label = ipm_.GenerateIPMImage(label_images);

    if(save_ipm_){
    	ros::Time timestamp = image0->header.stamp;
    	ROS_INFO("Timestamp: sec=%d, nsec=%d", timestamp.sec, timestamp.nsec);
    	std::string file_name = std::to_string(timestamp.sec) + "." + std::to_string(timestamp.nsec) + ".png";
        cv::imwrite(IPM_LABEL_DIR + file_name, ipm_label);
        cv::imwrite(IPM_RAW_DIR + file_name, ipm_img);
      ofs_train_csv_ << "./data/images/" + file_name << ",./data/labels/" + file_name << std::endl;
    }
//    cv::waitKey(1);
}
