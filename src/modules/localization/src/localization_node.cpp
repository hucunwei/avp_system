#include "ros/ros.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "localization_node");
    ros::NodeHandle nh("~");

    ros::spin();

}