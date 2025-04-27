//
// Created by ubuntu on 2024-12-31.
// Tong Qin: qintong@sjtu.edu.cn
//

#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include "avp_map.h"
#include "ros_viewer.h"
#include "planner.h"


std::vector<Slot> pure_slots;
TimedPose startPose;
TimedPose goalPose;
bool newGoalPoint = false;
Eigen::Vector2d goal(0, 0); // x,y
double goal_yaw;            // yaw
std::vector<TimedPose> readyPoses; // [readPose_A , readPose_B] for ROS markers
std::vector<Eigen::Vector2d> ready_2d_pos;
std::vector<double> ready_yaws;

// 全局变量：订阅器指针,方便 订阅一次 就立即 取消订阅
ros::Subscriber odom_sub;

double normalizeAngle(double radian){
    if(radian > M_PI) {return radian -= 2 * M_PI ;}
    if(radian < -M_PI) {return radian += 2 * M_PI ;}
    return radian;
}

void updateReadyPoses(const Slot &pure_slot) {
    ROS_INFO("Updating ready poses:");
    Eigen::Vector2d slot_center_2d(pure_slot.slot_center.x(), pure_slot.slot_center.y());
    Eigen::Vector3d slot_direction = pure_slot.corners_[1] - pure_slot.corners_[2];  // 计算 车位的 单位 方向向量
    Eigen::Vector2d slot_direction_2d(slot_direction.x(), slot_direction.y());
    slot_direction_2d.normalize(); // 归一化 方向向量
    
    // 计算 2d 左右 准备 位置
    Eigen::Vector2d ready_left = slot_center_2d + slot_direction_2d * 6.5;   //  6.5 = 靠边 距离1.8米+ 半车宽1.5 + 车位中心点到 入口的 3.2米 
    // 左转 90 度 , 对于方向向量 (x, y)，左转 90 度的公式是 = (-y, x)
    ready_left += Eigen::Vector2d(-slot_direction_2d.y(), slot_direction_2d.x()) * 6.3; // 6.3 = 车位 短边4.2 的1.5倍
    // 右转 90 度，对于方向向量 (x, y)，左转 90 度的公式是 = (y, -x)
    Eigen::Vector2d ready_right = slot_center_2d + slot_direction_2d * 6.5;   //  6.5 = 靠边 距离1.8米+ 半车宽1.5 + 车位中心点到 入口的 3.2米 
    ready_right += Eigen::Vector2d(slot_direction_2d.y(), -slot_direction_2d.x()) * 6.3; // 6.3 = 车位 短边4.2 的1.5倍
    // 存储 2d 左右 准备 位置
    ready_2d_pos.clear();
    ready_2d_pos.push_back(ready_left);
    ready_2d_pos.push_back(ready_right);

    // 计算 左右 yaw
    double left_yaw = normalizeAngle(pure_slot.yaw + M_PI / 2); // 左转 90 度
    double right_yaw = normalizeAngle(pure_slot.yaw - M_PI / 2); // 右转 90 度
    ready_yaws.clear();
    ready_yaws.push_back(left_yaw);
    ready_yaws.push_back(right_yaw);
    // 打印结果
    double left_degree = left_yaw * kToDeg;
    double right_degree = right_yaw * kToDeg;
    ROS_INFO("Left ready pose: x=%.2f, y=%.2f, yaw=%.2f, degree=%.2f", ready_left.x(), ready_left.y(), left_yaw,left_degree);
    ROS_INFO("Right ready pose: x=%.2f, y=%.2f, yaw=%.2f, degree=%.2f", ready_right.x(), ready_right.y(), right_yaw,right_degree);
    
    // 存储 左右 准备 位姿 for ROS visulization
    TimedPose readPose_left;
    Eigen::Vector3d left_position(ready_left.x(), ready_left.y(), 0);
    double rviz_left_yaw = normalizeAngle(left_yaw - M_PI/2);                      // 角度参考系变换： x 轴 上(竖直) 为 0   改成   x轴 左(水平) 为 0， 然后控制 yaw 在 -pi 和 pi 之间
    Eigen::Quaterniond left_orientation(Eigen::AngleAxisd(rviz_left_yaw, Eigen::Vector3d::UnitZ()));
    readPose_left.time_ = 0;
    readPose_left.t_ = left_position;      
    readPose_left.R_ = left_orientation;  
    readyPoses.clear();
    readyPoses.push_back(readPose_left);

    TimedPose readPose_right;
    Eigen::Vector3d right_position(ready_right.x(), ready_right.y(), 0);
    double rviz_right_yaw = normalizeAngle(right_yaw - M_PI/2);                      // 角度参考系变换： x 轴 上(竖直) 为 0   改成   x轴 左(水平) 为 0， 然后控制 yaw 在 -pi 和 pi 之间
    Eigen::Quaterniond right_orientation(Eigen::AngleAxisd(rviz_right_yaw, Eigen::Vector3d::UnitZ()));
    readPose_right.time_ = 0;
    readPose_right.t_ = right_position;
    readPose_right.R_ = right_orientation;
    readyPoses.push_back(readPose_right);
    
}

void navGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    double yaw = 0;

    // 提取位置信息并转换为 Eigen::Vector3d，  2D Nav Goal position 
    Eigen::Vector3d position(
        msg->pose.position.x,
        msg->pose.position.y,
        msg->pose.position.z
    );

    // 提取姿态信息并转换为 Eigen::Vector3d，  2D Nav Goal orientation 
    Eigen::Quaterniond orientation(
        msg->pose.orientation.w,  // 注意：w 在前
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z
    );

    ROS_INFO("Received 2D Nav Goal:");
    ROS_INFO("- Position: x=%.2f, y=%.2f, z=%.2f", position.x(), position.y(), position.z());
    ROS_INFO("- Orientation (Quaternion): w=%.2f, x=%.2f, y=%.2f, z=%.2f",
             orientation.w(), orientation.x(), orientation.y(), orientation.z());

    // ROS_INFO("pure_slots, there are %d slots", pure_slots.size());
    // readyPoses.clear(); // 如果 2D Nav Goal position 不是 停车位 内的点， 需要清空 准备车位

    for(size_t i=0 ; i < pure_slots.size(); ++i){
        if(pure_slots[i].bounding_box.contains(position)){
            ROS_INFO("Found the parking slot:");
            Eigen::Vector3d slot_center = pure_slots[i].slot_center;
            ROS_INFO("- slot_center: x=%.2f, y=%.2f, z=%.2f", slot_center.x(), slot_center.y(), slot_center.z());
            yaw = pure_slots[i].yaw;
            ROS_INFO("- yaw: %.2f", yaw);
            double degree = yaw * kToDeg;
            ROS_INFO("- degree: %.2f", degree);
            // 目标位姿（位置）修正为 停车位中心点：slot_center
            position = slot_center;
            // slot里的 yaw 是基于 x 轴 竖直， Rviz 里的 x轴  是 水平的
            double rviz_yaw = normalizeAngle(yaw - M_PI/2);                      // 角度参考系变换： x 轴 上(竖直) 为 0   改成   x轴 左(水平) 为 0， 然后控制 yaw 在 -pi 和 pi 之间
            // 目标位姿（姿态）修正为 停车位 开口方向：需要 rviz_yaw 转 Eigen::Quaterniond 格式
            Eigen::Quaterniond slot_orientation(Eigen::AngleAxisd(rviz_yaw, Eigen::Vector3d::UnitZ()));
            orientation = slot_orientation;

            // 更新 准备 车位 位姿
            updateReadyPoses(pure_slots[i]);
            break;
        }
    }
    // goal_state for path searching 
    newGoalPoint = true;
    goal.x() = position.x();
    goal.y() = position.y();  
    goal_yaw = yaw;

    // 存储 目标位姿（位置 + 姿态） 在 goalPose 
    goalPose.time_ = 0;
    goalPose.t_ = position;      // 2D Nav Goal position --> slot_center
    goalPose.R_ = orientation;      // 2D Nav Goal orientation --> slot_orientation
}

void navStartCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // 提取位置信息并转换为 Eigen::Vector3d
    Eigen::Vector3d position(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z
    );

    // 提取姿态信息并转换为 Eigen::Quaterniond
    Eigen::Quaterniond orientation(
        msg->pose.pose.orientation.w,  // 注意：w 在前
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z
    );

    // 打印结果
    ROS_INFO("Received Odom (from localization) Start:");
    ROS_INFO("- Position: x=%.2f, y=%.2f, z=%.2f", position.x(), position.y(), position.z());
    ROS_INFO("- Orientation (Quaternion): w=%.2f, x=%.2f, y=%.2f, z=%.2f",
             orientation.w(), orientation.x(), orientation.y(), orientation.z());
    
    // 存储 初始位姿（位置 + 姿态） 在 startPose 
    startPose.t_ = position;
    startPose.R_ = orientation;

    // 取消订阅，确保只接收一次消息
    odom_sub.shutdown();
    ROS_INFO("Planning Module: Unsubscribed from /localization/out_pose");
    
}

int main(int argc, char **argv) {
    // 初始化节点
    ros::init(argc, argv, "planning_node"); // 节点名字为 "planning_node"
    // 创建 私有命名空间的 nh("~")
    ros::NodeHandle nh("~");                // 在私有命名空间中 发布的相对话题名 会自动加上 节点名作为前缀, 如 /planning_node/topic_1， /planning_node/topic_2, ...

    // 读取 config/planning_config.yaml文件 文件路径 和 话题名 参数
    // 1. 初始化变量
    // std::string map_bin_path = "/home/wes/mvp_final/avp_system/src/modules/planning/data/avp_map.bin";      // 作业的 建图文件（测试）
    std::string map_bin_path = "/home/wes/mvp_final/avp_system/src/modules/planning/data/avp_map_sim.bin";  // 仿真器的 建图文件
    std::string navGoal = "/move_base_simple/goal";                      // Rivz 鼠标输入， 2D Navigation Goal
    std::string localization_odom = "/localization/out_pose";            // 通过 定位模块 实时语义定位， 提供 实时位姿， 出发点 start 需要 初始位姿 
    std::string truth_odom = "/gps_odom";                                // 通过 仿真器 实时定位， 提供 实时位姿， 出发点 start 需要 初始位姿 
    // 2. 从参数服务器读取参数，并提供默认值
    // nh.param<std::string>("map_bin_input_path",map_bin_path , "/home/wes/mvp_final/avp_system/src/modules/planning/data/avp_map.bin");
    nh.param<std::string>("map_bin_input_path",map_bin_path , "/home/wes/mvp_final/avp_system/src/modules/planning/data/avp_map_sim.bin");
    nh.param<std::string>("topics/navGoal", navGoal, "/move_base_simple/goal"); 
    nh.param<std::string>("topics/localization_odom", localization_odom, "/localization/out_pose");
    nh.param<std::string>("topics/truth_odom", truth_odom, "/gps_odom");

    // 订阅 话题
    ros::Subscriber sub1 = nh.subscribe(navGoal, 10, navGoalCallback);     // 订阅 2D Nav Goal, 终点位姿 goalPose 需要
    odom_sub = nh.subscribe(truth_odom, 10, navStartCallback);             // 订阅 实时定位， 初始位姿 startPose 需要

    // 加载 建图文件 
    AvpMap avp_map;
    avp_map.load(map_bin_path);
    // avp_map.load(DATASET_PATH "avp_map.bin");
    // avp_map.load(DATASET_PATH "avp_map_sim.bin");
    pure_slots = avp_map.getPureSlots();
    
    // 在 Rviz 可视化 avp map
    RosViewer rosViewer(nh);
    rosViewer.displayAvpMap(avp_map, 0);                    // 可视化 停车场的 路标点， 箭头点，（三边开口）车位点

    // 可视化 小车 的 初始位姿 startPose
    startPose.time_ = 0;                                    // 做 测试， 待删除
    startPose.t_= Eigen::Vector3d(19, 16, 0);               // 做 测试， 待删除
    startPose.R_ = Eigen::Quaterniond(1, 0, 0, 0);          // 做 测试， 待删除
    double startPose_yaw = GetYaw(startPose.R_);
    double startPose_degree = startPose_yaw * kToDeg;
    ROS_INFO("=== startPose_yaw = %d,  degree = ", startPose_yaw, startPose_degree);
    rosViewer.publishPose(startPose);

    // 生成 障碍点 obstacles
    // 1. 提取 （三边开口）车位点 作为 障碍点 obstacles
    std::vector<Eigen::Vector3d> obstaclePoints;
    for (const auto &p : avp_map.getSemanticElement((SemanticLabel::kSlot))) {
        Eigen::Vector3d tmp_p(p.x() * kPixelScale,
                p.y() * kPixelScale,
                p.z() * kPixelScale);
        obstaclePoints.push_back(tmp_p);
    }

    // 2. 障碍点 obstacles 转成 2D 障碍网格 grid coordinate, with resolution 0.5m*0.5m 
    double resolution = 2.0;    //1.5，1.0, 0.5
    Vector2i start_index(round(startPose.t_(0) / resolution),   // convert startPose to grid index
                   round(startPose.t_(1) / resolution));

    unordered_set<Vector2i, Vector2iHash> obstacles_index;      // convert obstaclePoints to grid index
    for (auto& p : obstaclePoints) {
        Vector2i obs_p(round(p.x() / resolution),
                     round(p.y() / resolution));
        obstacles_index.insert(obs_p);
    }

    ros::Rate loop_rate(10);
    while(ros::ok()) {
        loop_rate.sleep();
        ros::spinOnce();

        if(!newGoalPoint)
            continue;
        // visualize goal vehicle 2
        rosViewer.publishGoalPose(goalPose); 
        rosViewer.publishArrayPoses(readyPoses);

        newGoalPoint = false;
        Vector2i goal_index(round(goal.x() / resolution), round(goal.y() / resolution));    // convert newGoalPoint to grid index
        // Vector2i goal_index(round(goal_state.x / resolution), round(goal_state.y / resolution));
        

        // find path
        Planner planner;
        // // BFS
        // vector<Vector2i> path = planner.Bfs(start_index, goal_index, obstacles_index);
        // int Bfs_size = path.size();
        // ROS_INFO("BFS Planner:");
        // ROS_INFO("Position: BFS path.size() = %d", Bfs_size);
        // // publish result
        // if (path.empty()) {
        //     ROS_WARN("Fail to find BFS path");
        // } else {
        //     // path 的 每个点 是 grid 值 (x_index, y_index)，  把 grid 值 (x_index, y_index) 转成 真实值 （x,y） 
        //     std::vector<Eigen::Vector2d> path_points;
        //     for (auto &index : path) {
        //         Eigen::Vector2d tmp (index.x() * resolution, index.y() * resolution);
        //         path_points.emplace_back(tmp);
        //     }
        //     rosViewer.publishTrajectory(path_points);
        // }

        // The first part of Path: From startPose to readyPose
        // Using A*  
        TimedPose readyPoses_right = readyPoses[1];     // TimedPose readyPoses_left = readyPoses[0];
        Vector2i ready_index(round(readyPoses_right.t_(0) / resolution), round(readyPoses_right.t_(1) / resolution));  // convert startPose to grid index
        vector<Vector2i> pathA = planner.AStar(start_index, ready_index, obstacles_index);
        int AStar_size = pathA.size();
        ROS_INFO("AStar Planner:");
        ROS_INFO("Position: [AStar path.size() = %d", AStar_size);
        // publish result
        if (pathA.empty()) {
            ROS_WARN("Fail to find A* path");
        } else {
            // pathA 的 每个点 是 grid 值 (x_index, y_index)，  把 grid 值 (x_index, y_index) 转成 真实值 （x,y） 
            std::vector<Eigen::Vector2d> path_points;
            for (auto &index : pathA) {
                Eigen::Vector2d tmp (index.x() * resolution, index.y() * resolution);
                path_points.emplace_back(tmp);
            }
            rosViewer.publishATrajectory(path_points);
        }

        // The second part of Path: From readyPose to goalPose
        // using A*
        vector<Vector2i> pathA2 = planner.AStar(ready_index, goal_index, obstacles_index);
        int AStar_2_size = pathA2.size();
        ROS_INFO("AStar Planner 2:");
        ROS_INFO("Position: [AStar pathA2.size() = %d", AStar_2_size);
        // publish result
        if (pathA2.empty()) {
            ROS_WARN("Fail to find A* path 2");
        } else {
            // pathA 的 每个点 是 grid 值 (x_index, y_index)，  把 grid 值 (x_index, y_index) 转成 真实值 （x,y） 
            std::vector<Eigen::Vector2d> path_points;
            for (auto &index : pathA2) {
                Eigen::Vector2d tmp (index.x() * resolution, index.y() * resolution);
                path_points.emplace_back(tmp);
            }
            rosViewer.publishHybridATrajectory(path_points);
        }

        // // The second part of Path: From readyPose to goalPose
        // // using hybrid A*
        // double wheelbase = 2.5;          // 轴距
        // double step_size = resolution;   // 步长 (= 速度 * 单位时间) 0.5，1.0，1.5 让 step_size = 障碍物的尺寸 resolution 
        // double max_steer = M_PI / 4;     // 最大转向角（45度）
        // double readyPoses_yaw = GetYaw(readyPoses_right.R_);
        // ROS_INFO("=== readyPoses_yaw = %d", readyPoses_yaw);
        // double readyPoses_degree = readyPoses_yaw * kToDeg;
        // ROS_INFO("=== readyPoses_yaw = %d,  degree = ", readyPoses_yaw, readyPoses_degree);    
        // State startState(readyPoses_right.t_(0), readyPoses_right.t_(1), readyPoses_yaw, 0, 0, nullptr);
        // State goalState(goal.x(), goal.y(), goal_yaw, 0, 0, nullptr);
        // std::vector<State*> hybridApath = planner.HybridAStar(startState, goalState,
        //                                                        obstacles_index,
        //                                                         wheelbase, step_size, max_steer);
        // // publish result
        // if (hybridApath.empty()) {
        //     ROS_WARN("Fail to find Hybrid A* path");
        // } else {
            
        //     std::vector<Eigen::Vector2d> path_points;
        //     for (auto &pState : hybridApath) {
        //         Eigen::Vector2d tmp (pState->x, pState->y);
        //         path_points.emplace_back(tmp);
        //     }
        //     rosViewer.publishHybridATrajectory(path_points);
        // }



    }

    ros::spin();
}