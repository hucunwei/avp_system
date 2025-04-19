#pragma once

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


class RosViewer : public ViewerInterface {
    public:
     RosViewer(ros::NodeHandle &n, bool imshow, bool autorun);
     ~RosViewer();
   
     void displayAvpMap(const TimedPose &pose, const Map &avp_map, const std::vector<Slot> &cur_slots) override;
   
    private:
     void publishSlots(const std::vector<Slot> &slots, double time, ros::Publisher &publisher, bool avp_slots);
   
     void publishPoints(const std::vector<Eigen::Vector3i> &grid, double time, ros::Publisher &publisher);
   
     void publishPose(const TimedPose &pose);
   
     ros::NodeHandle &nh_;
     nav_msgs::Path path_; // trajectory
     ros::Publisher pub_path_, pub_odometry_, pub_mesh_;
     ros::Publisher pub_global_dash_pts_, pub_global_arrow_pts_, pub_global_slot_pts_;
     ros::Publisher pub_slot_marker_, pub_slot_marker_ipm_;
   };