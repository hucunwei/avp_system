//
// Created by ubuntu on 25-3-1.
//
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <Eigen/Dense>

#include <visualization_msgs/MarkerArray.h>
#include "avp_map.h"
#include "slot.h"

#ifndef AVP_WS_ROS_VIEWER_H
#define AVP_WS_ROS_VIEWER_H
class RosViewer {
public:
    RosViewer(ros::NodeHandle &n);
    // new add
    // void displayAvpMap(const AvpMap &avp_map, double time);
    void publishPoints(const std::vector<Eigen::Vector3f> &points, double time);

    // old
    void publishPose(const double x, const double y, const double yaw);

    void publishReferencePoint(const double x, const double y);

    void publishTrajectory(const std::vector<std::vector<double>> &trajectory);

// private:
    // void publishSlots(const std::vector<Slot> &slots, double time, ros::Publisher &publisher, bool avp_slots);
    void publishPoints(const std::vector<Eigen::Vector3i> &grid, double time, ros::Publisher &publisher);
    nav_msgs::Path path_, reference_;
    ros::Publisher pub_path_, pub_pose_, pub_mesh_, pub_reference_, pub_ref_pts_ ;
    // ros::Publisher pub_global_dash_pts_, pub_global_arrow_pts_, pub_global_slot_pts_, pub_slot_marker_, pub_current_pts_;
};
#endif //AVP_WS_ROS_VIEWER_H
