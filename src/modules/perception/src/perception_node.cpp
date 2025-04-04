#include "ros/ros.h"
#include <iostream>
#include <string>

int main(int argc, char **argv) {
    ros::init(argc, argv, "perception_node");
    ros::NodeHandle nh("~");

    std::string camera_topic;
    nh.param("topics/camera", camera_topic, std::string("/camera")); 
    std::cout << camera_topic << std::endl;

    ros::spin();

}