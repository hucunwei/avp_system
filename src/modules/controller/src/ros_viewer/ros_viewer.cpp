//
// Created by ubuntu on 25-3-1.
//

#include "ros_viewer.h"

geometry_msgs::Point to_ros(const Eigen::Vector3d &pt) {
    geometry_msgs::Point ret;
    ret.x = pt.x();
    ret.y = pt.y();
    ret.z = pt.z();
    return ret;
}

RosViewer::RosViewer(ros::NodeHandle &n){
    pub_path_ = n.advertise<nav_msgs::Path>("path", 1000, true);
    pub_reference_ = n.advertise<nav_msgs::Path>("reference", 1000, true);
    pub_pose_ = n.advertise<nav_msgs::Odometry>("pose", 1000, true);
    pub_mesh_ = n.advertise<visualization_msgs::Marker>("vehicle", 100, true);
    pub_ref_pts_ = n.advertise<sensor_msgs::PointCloud>("reference_pts", 100, true);
    path_.header.frame_id = "world";
    reference_.header.frame_id = "world";
    // new add
    // pub_slot_marker_ = n.advertise<visualization_msgs::Marker>("slot_vector", 100, true);
    // pub_global_dash_pts_ = n.advertise<sensor_msgs::PointCloud>("global_dash_pts", 100, true);
    // pub_global_arrow_pts_ = n.advertise<sensor_msgs::PointCloud>("global_arrow_pts", 100, true);
    // pub_global_slot_pts_ = n.advertise<sensor_msgs::PointCloud>("global_slot_pts", 100, true);
    // pub_current_pts_ = n.advertise<sensor_msgs::PointCloud>("current_pts", 100, true);
}
    
    

// void RosViewer::displayAvpMap(const AvpMap &avp_map, double time) {
//     // publish slot
//     publishSlots(avp_map.getAllSlots(), time, pub_slot_marker_, true);
//     // publish semantic points
//     auto grid_slot = avp_map.getSemanticElement(SemanticLabel::kSlot);
//     publishPoints({grid_slot.begin(), grid_slot.end()}, time, pub_global_slot_pts_);
//     auto grid_arrow = avp_map.getSemanticElement(SemanticLabel::kArrowLine);
//     publishPoints({grid_arrow.begin(), grid_arrow.end()}, time, pub_global_arrow_pts_);
//     auto grid_dash = avp_map.getSemanticElement(SemanticLabel::kDashLine);
//     publishPoints({grid_dash.begin(), grid_dash.end()}, time, pub_global_dash_pts_);

//     ros::spinOnce();
// }

// void RosViewer::publishSlots(const std::vector<Slot> &slots, double time, ros::Publisher &publisher, bool avp_slots) {
//     if (slots.empty()) {
//         return;
//     }
//     visualization_msgs::Marker line_list;
//     line_list.header.frame_id = "world";
//     line_list.header.stamp = ros::Time(time);
//     line_list.ns = "lines";
//     line_list.action = visualization_msgs::Marker::ADD;
//     line_list.pose.orientation.w = 1.0;
//     line_list.type = visualization_msgs::Marker::LINE_LIST;
//     line_list.scale.x = 0.1;
//     line_list.id = 0;

//     if (avp_slots) {
//         line_list.color.g = 1.0;
//     } else {
//         line_list.color.r = 1.0;
//     }
//     line_list.color.a = 1.0;

//     for (const auto &slot: slots) {
//         auto p0 = to_ros(slot.corners_[0]);
//         auto p1 = to_ros(slot.corners_[1]);
//         auto p2 = to_ros(slot.corners_[2]);
//         auto p3 = to_ros(slot.corners_[3]);
//         line_list.points.push_back(p0);
//         line_list.points.push_back(p1);
//         line_list.points.push_back(p1);
//         line_list.points.push_back(p2);
//         line_list.points.push_back(p2);
//         line_list.points.push_back(p3);
//         line_list.points.push_back(p3);
//         line_list.points.push_back(p0);
//     }
//     publisher.publish(line_list);
// }

// void RosViewer::publishPoints(const std::vector<Eigen::Vector3f> &points, double time) {
//     sensor_msgs::PointCloud global_cloud;
//     global_cloud.header.frame_id = "world";
//     global_cloud.header.stamp = ros::Time(time);
//     for (Eigen::Vector3f pt: points) {
//         geometry_msgs::Point32 p;
//         p.x = pt.x();
//         p.y = pt.y();
//         p.z = pt.z();
//         global_cloud.points.push_back(p);
//     }
//     pub_current_pts_.publish(global_cloud);
// }

// void RosViewer::publishPoints(const std::vector<Eigen::Vector3i> &grid, double time, ros::Publisher &publisher) {
//     sensor_msgs::PointCloud global_cloud;
//     global_cloud.header.frame_id = "world";
//     global_cloud.header.stamp = ros::Time(time);
//     for (Eigen::Vector3i pt: grid) {
//         geometry_msgs::Point32 p;
//         p.x = pt.x() * kPixelScale;
//         p.y = pt.y() * kPixelScale;
//         p.z = pt.z() * kPixelScale;
//         global_cloud.points.push_back(p);
//     }
//     printf("publish points %d \n", global_cloud.points.size());
//     publisher.publish(global_cloud);
// }



void RosViewer::publishReferencePoint(const double x, const double y) {
    sensor_msgs::PointCloud global_cloud;
    global_cloud.header.frame_id = "world";
    global_cloud.header.stamp = ros::Time(0);

    geometry_msgs::Point32 p;
    p.x = x;
    p.y = y;
    p.z = 0;
    global_cloud.points.push_back(p);
    pub_ref_pts_.publish(global_cloud);
}

void RosViewer::publishPose(const double x, const double y, const double yaw) {
    Eigen::Vector3d position = Eigen::Vector3d(x, y, 0);
    Eigen::Matrix3d R;
    R << cos(yaw), -sin(yaw), 0,
    sin(yaw), cos(yaw), 0,
    0, 0, 1;
    Eigen::Quaterniond q;
    q = R;

    // pub odometry
    nav_msgs::Odometry odometry;
    odometry.header.frame_id = "world";
    odometry.header.stamp = ros::Time(0);
    odometry.pose.pose.position.x = position(0);
    odometry.pose.pose.position.y = position(1);
    odometry.pose.pose.position.z = position(2);
    odometry.pose.pose.orientation.x = q.x();
    odometry.pose.pose.orientation.y = q.y();
    odometry.pose.pose.orientation.z = q.z();
    odometry.pose.pose.orientation.w = q.w();
    pub_pose_.publish(odometry);

    // pub path（累加）
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";
    pose_stamped.header.stamp = ros::Time(0);
    pose_stamped.pose = odometry.pose.pose;
    path_.poses.push_back(pose_stamped);
    if (path_.poses.size() > 10000) {
        path_.poses.erase(path_.poses.begin());
    }
    pub_path_.publish(path_);
    // pub tf
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion tf_q;
    transform.setOrigin(tf::Vector3(position(0),
                                    position(1),
                                    position(2)));
    tf_q.setW(q.w());
    tf_q.setX(q.x());
    tf_q.setY(q.y());
    tf_q.setZ(q.z());
    transform.setRotation(tf_q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time(0),
                                          "world", "vehicle"));
    // pub Mesh model
    visualization_msgs::Marker meshROS;
    meshROS.header.frame_id = std::string("world");
    meshROS.header.stamp = ros::Time(0);
    meshROS.ns = "mesh";
    meshROS.id = 0;
    meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
    meshROS.action = visualization_msgs::Marker::ADD;
    Eigen::Matrix3d frontleftup2rightfrontup;
    frontleftup2rightfrontup << 0, 1, 0, -1, 0, 0, 0, 0, 1;
    Eigen::Matrix3d rot_mesh;
    rot_mesh << -1, 0, 0, 0, 0, 1, 0, 1, 0;
    Eigen::Quaterniond q_mesh;
    q_mesh = q * rot_mesh;
    Eigen::Vector3d t_mesh = position;
    meshROS.pose.orientation.w = q_mesh.w();
    meshROS.pose.orientation.x = q_mesh.x();
    meshROS.pose.orientation.y = q_mesh.y();
    meshROS.pose.orientation.z = q_mesh.z();
    meshROS.pose.position.x = t_mesh(0);
    meshROS.pose.position.y = t_mesh(1);
    meshROS.pose.position.z = t_mesh(2);
    meshROS.scale.x = 1.0;
    meshROS.scale.y = 1.0;
    meshROS.scale.z = 1.0;
    meshROS.color.a = 1.0;
    meshROS.color.r = 1.0;
    meshROS.color.g = 0.0;
    meshROS.color.b = 0.0;
    // std::string mesh_resource(MESH_PATH);
    std::string mesh_resource = "package://controller/meshes/car.dae";
    meshROS.mesh_resource = mesh_resource;
    pub_mesh_.publish(meshROS);
}

void RosViewer::publishTrajectory(const std::vector<std::vector<double>> &trajectory) {
    // pub path
    reference_.poses.clear();
    for (auto &p : trajectory) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "world";
        pose_stamped.header.stamp = ros::Time(0);
        pose_stamped.pose.position.x = p[0];
        pose_stamped.pose.position.y = p[1];
        pose_stamped.pose.position.z = 0;
        reference_.poses.push_back(pose_stamped);
    }
    pub_reference_.publish(reference_);
}