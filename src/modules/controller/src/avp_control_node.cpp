//
// Created by ubuntu on 2024-12-31.
// Tong Qin: qintong@sjtu.edu.cn
//

#include "ros/ros.h"
#include "ros_viewer.h"
#include "controller.h"
#include "KinematicModel.h"
// new add
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <lgsvl_msgs/CanBusData.h> // 包含 CanBusData 消息类型
#include <lgsvl_msgs/VehicleControlData.h> // 包含 VehicleControlData 消息类型

#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "visualization_msgs/Marker.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include "avp_map.h"


bool newPath = false;
vector<vector<double>> planning_path_1;
vector<double> goal_1_pos(2); // goal_pos = [x, y]
vector<vector<double>> planning_path_2;
vector<double> goal_2_pos(2); // goal2_pos = [x, y]
KinematicModel vehicle_model(0, -3, 0, 1, 2, 0.1);  // x, y, psi, v, L, dt
VehicleChassis vehicle_chassis(0, 0, 0, 0, 0, 0); // speed_mps, selected_gear, throttle_pct, brake_pct, steer_pct

vector<vector<double>> generateRefTrajectory() {
    vector<vector<double>> refer_path(1000, vector<double>(3)); // 1000 points, every point: (x,y,yaw) = (  x(t), y(t), yaw(t) ) 
    vector<double> refer_x, refer_y;
    // generate reference path
    for (int i = 0; i < 1000; i++) {
        refer_path[i][0] = 0.1 * i;
        refer_path[i][1] = 2 * sin(refer_path[i][0] / 3.0) + 2.5 * cos(refer_path[i][0] / 2.0);
        refer_x.push_back(refer_path[i][0]);
        refer_y.push_back(refer_path[i][1]);
    }
    double dx,dy,ddx,ddy;
    for(int i=0;i<refer_path.size();i++){
        if (i==0){
            dx = refer_path[i+1][0] - refer_path[i][0];
            dy = refer_path[i+1][1] - refer_path[i][1];
            ddx = refer_path[2][0] + refer_path[0][0] - 2*refer_path[1][0];
            ddy = refer_path[2][1] + refer_path[0][1] - 2*refer_path[1][1];
        }else if(i==refer_path.size()-1){
            dx = refer_path[i][0] - refer_path[i-1][0];
            dy = refer_path[i][1] - refer_path[i-1][1];
            ddx = refer_path[i][0] + refer_path[i-2][0] - 2*refer_path[i-1][0];
            ddy = refer_path[i][1] + refer_path[i-2][1] - 2*refer_path[i-1][1];
        }else{
            dx = refer_path[i+1][0] - refer_path[i][0];
            dy = refer_path[i+1][1] - refer_path[i][1];
            ddx = refer_path[i+1][0] + refer_path[i-1][0] - 2*refer_path[i][0];
            ddy = refer_path[i+1][1] + refer_path[i-1][1] - 2*refer_path[i][1];
        }
        refer_path[i][2] = atan2(dy,dx);//yaw
    }
    return refer_path;
}

// 定义计算欧式距离的函数
double distance(double x1, double y1, double x2, double y2) {
    // 使用欧几里得距离公式：sqrt((x2 - x1)^2 + (y2 - y1)^2)
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));

    // 使用曼哈顿距离公式：|x2 - x1| + |y2 - y1|
    // return std::abs(x2 - x1) + std::abs(y2 - y1);
}

double normalizeAngle2(double angle) {
    while (angle > PI) {
        angle -= 2.0 * PI;
    }
    while (angle < -PI) {
        angle += 2.0 * PI;
    }
    return angle;
}


// 回调函数：当接收到 nav_msgs::Path 消息时执行
void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    // 打印路径的基本信息
    ROS_INFO("---------   pathCallback: Received a new Path message with %zu poses.   ---------", msg->poses.size());
    vector<vector<double>> new_refer_path(msg->poses.size(), vector<double>(3)); // msg->poses.size() poses, every poses -> (x,y,yaw) = (  x(t), y(t), yaw(t) ) 

    // 遍历路径中的每个姿态点
    for (size_t i = 0; i < msg->poses.size(); ++i) {
        const geometry_msgs::PoseStamped& pose = msg->poses[i];
        // ROS_INFO("Pose %zu: Position (x: %.2f, y: %.2f, z: %.2f), Orientation (w: %.2f)", i, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.w);
        new_refer_path[i][0] = pose.pose.position.x;
        new_refer_path[i][1] = pose.pose.position.y;

        // 计算方向（yaw）
        double dx,dy,ddx,ddy;
        if (i==0){
            dx = new_refer_path[i+1][0] - new_refer_path[i][0];
            dy = new_refer_path[i+1][1] - new_refer_path[i][1];
            ddx = new_refer_path[2][0] + new_refer_path[0][0] - 2*new_refer_path[1][0];
            ddy = new_refer_path[2][1] + new_refer_path[0][1] - 2*new_refer_path[1][1];
        }else if(i==new_refer_path.size()-1){
            dx = new_refer_path[i][0] - new_refer_path[i-1][0];
            dy = new_refer_path[i][1] - new_refer_path[i-1][1];
            ddx = new_refer_path[i][0] + new_refer_path[i-2][0] - 2*new_refer_path[i-1][0];
            ddy = new_refer_path[i][1] + new_refer_path[i-2][1] - 2*new_refer_path[i-1][1];
        }else{
            dx = new_refer_path[i+1][0] - new_refer_path[i][0];
            dy = new_refer_path[i+1][1] - new_refer_path[i][1];
            ddx = new_refer_path[i+1][0] + new_refer_path[i-1][0] - 2*new_refer_path[i][0];
            ddy = new_refer_path[i+1][1] + new_refer_path[i-1][1] - 2*new_refer_path[i][1];
        }
        new_refer_path[i][2] = atan2(dy,dx);//yaw

    }
    // update new refer_path
    planning_path_1.clear();
    planning_path_1 = new_refer_path;

    goal_1_pos[0] = new_refer_path[msg->poses.size()-1][0];    // last pose x
    goal_1_pos[1] = new_refer_path[msg->poses.size()-1][1];    // last pose y
    ROS_INFO("Finshed loading the new Path message.");  
    newPath = true;
}


// 回调函数：当接收到 nav_msgs::Path 消息时执行
void path2Callback(const nav_msgs::Path::ConstPtr& msg) {
    // 打印路径的基本信息
    ROS_INFO("---------   pathCallback: Received a new Path message with %zu poses.   ---------", msg->poses.size());
    vector<vector<double>> new_refer_path(msg->poses.size(), vector<double>(3)); // msg->poses.size() poses, every poses -> (x,y,yaw) = (  x(t), y(t), yaw(t) ) 

    // 遍历路径中的每个姿态点
    for (size_t i = 0; i < msg->poses.size(); ++i) {
        const geometry_msgs::PoseStamped& pose = msg->poses[i];
        // ROS_INFO("Pose %zu: Position (x: %.2f, y: %.2f, z: %.2f), Orientation (w: %.2f)", i, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.w);
        new_refer_path[i][0] = pose.pose.position.x;
        new_refer_path[i][1] = pose.pose.position.y;

        // 计算方向（yaw）
        double dx,dy,ddx,ddy;
        if (i==0){
            dx = new_refer_path[i+1][0] - new_refer_path[i][0];
            dy = new_refer_path[i+1][1] - new_refer_path[i][1];
            ddx = new_refer_path[2][0] + new_refer_path[0][0] - 2*new_refer_path[1][0];
            ddy = new_refer_path[2][1] + new_refer_path[0][1] - 2*new_refer_path[1][1];
        }else if(i==new_refer_path.size()-1){
            dx = new_refer_path[i][0] - new_refer_path[i-1][0];
            dy = new_refer_path[i][1] - new_refer_path[i-1][1];
            ddx = new_refer_path[i][0] + new_refer_path[i-2][0] - 2*new_refer_path[i-1][0];
            ddy = new_refer_path[i][1] + new_refer_path[i-2][1] - 2*new_refer_path[i-1][1];
        }else{
            dx = new_refer_path[i+1][0] - new_refer_path[i][0];
            dy = new_refer_path[i+1][1] - new_refer_path[i][1];
            ddx = new_refer_path[i+1][0] + new_refer_path[i-1][0] - 2*new_refer_path[i][0];
            ddy = new_refer_path[i+1][1] + new_refer_path[i-1][1] - 2*new_refer_path[i][1];
        }
        new_refer_path[i][2] = atan2(dy,dx);//yaw

    }
    // update new refer_path
    planning_path_2.clear();
    planning_path_2 = new_refer_path;

    goal_2_pos[0] = new_refer_path[msg->poses.size()-1][0];    // last pose x
    goal_2_pos[1] = new_refer_path[msg->poses.size()-1][1];    // last pose y
    ROS_INFO("Finshed loading the new Path 2 message.");  
    newPath = true;
}

// 回调函数：当接收到 nav_msgs::Odometry 消息时执行
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // 打印里程计的基本信息
    ROS_INFO("---------   odomCallback: Received Odometry message:   ---------");

    // 提取位置信息
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;

    // 提取方向信息（四元数）
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    // 转换四元数为欧拉角（偏航角 yaw）
    tf::Quaternion quat(qx, qy, qz, qw);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // 打印位置和方向信息
    ROS_INFO("Position: (x: %.2f, y: %.2f, z: %.2f)", x, y, z);
    ROS_INFO("Orientation (Roll, Pitch, Yaw): (%.2f, %.2f, %.2f)", roll, pitch, yaw);

    // 提取线速度和角速度信息
    double linear_x = msg->twist.twist.linear.x;
    double angular_z = msg->twist.twist.angular.z;

    // 打印速度信息
    ROS_INFO("Linear Velocity (x): %.2f", linear_x);
    ROS_INFO("Angular Velocity (z): %.2f", angular_z);

    // 更新状态
    vehicle_model.x = x;
    vehicle_model.y = y;
    vehicle_model.psi = yaw;
    ROS_INFO("Finshed updating vehicle_model initial state  (%.2f, %.2f, %.2f)", vehicle_model.x,vehicle_model.y ,vehicle_model.psi );
}

// 回调函数：处理 /chassis 主题的消息
void chassisCallback(const lgsvl_msgs::CanBusData::ConstPtr& msg) {
    ROS_INFO("---------   chassisCallback: Received CanBusData message:   ---------");

    // 提取车速信息（假设字段名为 speed）
    vehicle_chassis.speed_mps = msg->speed_mps;         // 车速
    vehicle_chassis.selected_gear = msg->selected_gear; // 变速器档位
    vehicle_chassis.throttle_pct = msg->throttle_pct;   // 油门踏板开度
    vehicle_chassis.brake_pct = msg->brake_pct;         // 刹车踏板开度
    vehicle_chassis.steer_pct = msg->steer_pct;         // 方向盘转角
    vehicle_chassis.steer_angle = msg->steer_pct * (M_PI / 4); // 将 steer_pct 转换为弧度范围 [-pi/4, pi/4]
    // 打印接收到的数据
    ROS_INFO("Received Vehicle Speed: %.2f m/s", vehicle_chassis.speed_mps);
    ROS_INFO("Received Selected Gear: %d", vehicle_chassis.selected_gear);
    ROS_INFO("Received Throttle Percentage: %.2f", vehicle_chassis.throttle_pct);
    ROS_INFO("Received Brake Percentage: %.2f", vehicle_chassis.brake_pct);
    ROS_INFO("Received Steering Percentage: %.2f", vehicle_chassis.steer_pct);
    ROS_INFO("Received Steering Angle: %.2f rad", vehicle_chassis.steer_angle);

    // 更新 vehicle_model
    vehicle_model.v = vehicle_chassis.speed_mps;        // 更新车速
    
}

int main(int argc, char **argv) {
    // 初始化节点
    ros::init(argc, argv, "controller_node"); // 节点名字为 "control_node"
    // 创建 私有命名空间的 nh("~")
    ros::NodeHandle nh("~");               // 在私有命名空间中 发布的相对话题名 会自动加上 节点名作为前缀, 如 /controller_node/topic_1， /controller_node/topic_2, ...
    // 创建一个 RosViewer 对象，用于可视化
    RosViewer rosviewer(nh); 

    // 读取 config/planning_config.yaml文件 文件路径 和 话题名 参数


    // 接收 规划路径， 订阅 规划路径 话题消息
    ros::Subscriber path_1_sub = nh.subscribe<nav_msgs::Path>("/planning_node/planned_A_path", 10, pathCallback);
    ros::Subscriber path_2_sub = nh.subscribe<nav_msgs::Path>("/planning_node/planned_hybridA_path", 10, path2Callback);

    // 接收 定位(位姿), 订阅 里程计 话题消息
    // ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/localization/out_pose", 10, odomCallback);       // 订阅 定位模块（语义定位 里程计）
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/gps_odom", 10, odomCallback);                     // 订阅 仿真器 里程计信息
    // ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/planning_node/odometry", 10, odomCallback);       // 订阅 规划模块 里程计信息 （测试用的）

    // 接收 车辆底盘信息， 订阅 底盘信息 话题消息
    ros::Subscriber chassis_sub = nh.subscribe<lgsvl_msgs::CanBusData>("/chassis", 10, chassisCallback);// 订阅 底盘信息


    // 创建发布者，发布到 /vehicle_cmd 主题
    ros::Publisher control_pub = nh.advertise<lgsvl_msgs::VehicleControlData>("/vehicle_cmd", 10);
    // 创建 VehicleControlData 消息
    lgsvl_msgs::VehicleControlData control_msg;

    Controller controller;


    // Intital state [0, -1, 0.5]
    // KinematicModel vehicle_model(0, -3, 0, 2, 2, 0.1);  // x, y, psi, v, L, dt
    // double rviz_yaw = normalizeAngle2(vehicle_model.psi + M_PI/2);               // 角度参考系变换： x 轴 上(竖直) 为 0   改成   x轴 右(水平) 为 0， 然后控制 yaw 在 -pi 和 pi 之间
    // rosviewer.publishPose(vehicle_model.x, vehicle_model.y, vehicle_model.psi); // 发布初始 位置

    vector<double> robot_state(4);
    ros::Rate loop_rate(10);
    while(ros::ok()) {
        loop_rate.sleep();
        ros::spinOnce();

        

        if(!newPath)
            continue;
        // reset newPath
        newPath = false;

        // visualize vehicle start pose
        double rviz_yaw = normalizeAngle2(vehicle_model.psi + M_PI/2);      // 角度参考系变换： x 轴 上(竖直) 为 0   改成   x轴 右(水平) 为 0， 然后控制 yaw 在 -pi 和 pi 之间
        rosviewer.publishPose(vehicle_model.x, vehicle_model.y, rviz_yaw);  // 发布初始 位置
        
        

        // path 1
        // 读取 底盘信息 chassis 
        double current_distance = distance(vehicle_model.x, vehicle_model.y, goal_1_pos[0], goal_1_pos[1]);
        double current_speed = vehicle_chassis.speed_mps; // longitudinal_control = 1 等于 velocity 2.0m/s, 速度
        
        // longitudinal_control 
        double acceleration_pct = 0.0; // 油门踏板开度
        double braking_pct = 0.0;      // 刹车踏板开度
        // lateral_control
        double target_wheel_angle = 0.0;                    // 方向盘转角 radians
        double target_wheel_angular_rate = (M_PI / 4)/2;    // 方向盘转速 radians / second, 2 秒 转 M_PI / 4
        // other control
        control_msg.target_gear = 1;                      // 1:前进档, 2:倒车档, 3:泊车档
        


        while( current_distance > 0.1) {
            ROS_INFO("The vehicle is moving toward the goal position 1. distance is: %.2f, the current speed is: %.2f ", current_distance, current_speed);
            // 计算纵向控制（前行速度）：  longitudinal_control
            if(current_distance > 10 ){  
                // > 10 米, 加速， 油门踩到底 acceleration_pct
                acceleration_pct = 1;   // add max acceleration
            }else if(current_distance > 5){ 
                // 5 ~ 10 米, 维持低速， 油门踩少一点
                acceleration_pct = 0.3;  // add some acceleration to remain low speed
            }else{
                // <= 5 米, 超低速， 油门踩少少一点
                acceleration_pct  = 0.1;  // add little acceleration to remain super low speed
            }
            robot_state[0] = vehicle_model.x;
            robot_state[1] = vehicle_model.y;
            robot_state[2] = vehicle_model.psi;
            robot_state[3] = vehicle_model.v;

            // 计算横向控制（方向盘 转角）： 
            // lateral_control = controller.PIDController(robot_state, planning_path_1, vehicle_model);
            // lateral_control = controller.StanlyController(robot_state, planning_path_1, vehicle_model);
            target_wheel_angle = controller.PurePursuitController(robot_state, planning_path_1, vehicle_model);
            // lateral_control = controller.LQRController(robot_state, planning_path_1, vehicle_model);
            
            // 给 仿真器 的 发送 控制信号（横向控制 和 纵向控制）
            control_msg.acceleration_pct = acceleration_pct;                    // 油门踏板开度
            control_msg.braking_pct = braking_pct;                              // 刹车踏板开度
            control_msg.target_wheel_angle = target_wheel_angle;                // 方向盘转角  
            control_msg.target_wheel_angular_rate = target_wheel_angular_rate;  // 方向盘转速
            

            // 给 仿真器 的 1/10 秒 时间 执行到 预期的控制信号 
            control_pub.publish(control_msg);   // 发布控制信号
            loop_rate.sleep();                  // 确保循环以 10 Hz 的频率运行

            // visualize path
            // rviz_yaw = normalizeAngle2(vehicle_model.psi + M_PI/2); 
            rosviewer.publishPose(vehicle_model.x, vehicle_model.y, vehicle_model.psi);
            current_distance = distance(vehicle_model.x, vehicle_model.y, goal_1_pos[0], goal_1_pos[1]);
            current_speed = vehicle_chassis.speed_mps;
            
        }
        // 发送 控制信号， 刹车，换倒挡
        ROS_INFO("--- The vehicle is arrived at the goal position 1. with distance is: %.2f", current_distance);
        

        // 刹车 和 换挡
        control_msg.braking_pct = 1.0;                    // 刹车踏板开度
        control_msg.acceleration_pct = 0;                 // 油门踏板开度
        control_msg.target_wheel_angle = 0.0;             // 方向盘转角 radians
        control_pub.publish(control_msg);                 // 发布控制信号： 刹车
        loop_rate.sleep();                                // 确保循环以 10 Hz 的频率运行， 给 仿真器 的 1/10 秒 时间 执行到 预期的控制信号 

        control_msg.target_gear = 2;                      // 1:前进档, 2:倒车档, 3:泊车档
        control_pub.publish(control_msg);                 // 发布控制信号: 换挡

        // path 2
        // 读取 底盘信息 chassis 
        current_distance = distance(vehicle_model.x, vehicle_model.y, goal_2_pos[0], goal_2_pos[1]);
        current_speed = vehicle_chassis.speed_mps;
        // longitudinal_control 
        acceleration_pct = 0.0; // 油门踏板开度
        braking_pct = 0.0;      // 刹车踏板开度
        // lateral_control
        target_wheel_angle = 0.0;                    // 方向盘转角 radians
        target_wheel_angular_rate = (M_PI / 4)/2;    // 方向盘转速 radians / second, 2 秒 转 M_PI / 4



        // longitudinal_control = -1; // constant velocity 2.0m/
        while( current_distance > 0.1) {
            ROS_INFO("The vehicle is moving toward the goal position 2. distance is: %.2f, the current speed is: %.2f ", current_distance, current_speed);
            // 计算纵向控制（前行速度）：  longitudinal_control
            if(current_distance > 3 ){  
                // > 3 米, 加速， 油门踩到底 acceleration_pct
                acceleration_pct = 0.5;   
            }else if(current_distance > 1){ 
                // 5 ~ 10 米, 维持低速， 油门踩少一点
                acceleration_pct = 0.1;  // add some acceleration to remain low speed
            }else{
                // <= 5 米, 超低速， 油门踩少少一点
                acceleration_pct  = 0.05;  // add little acceleration to remain super low speed
            }

            robot_state[0] = vehicle_model.x;
            robot_state[1] = vehicle_model.y;
            robot_state[2] = vehicle_model.psi;
            robot_state[3] = vehicle_model.v;

            // lateral_control = controller.PIDController(robot_state, planning_path_2, vehicle_model);
            // lateral_control = controller.StanlyController(robot_state, planning_path_2, vehicle_model);
            target_wheel_angle = controller.PurePursuitController(robot_state, planning_path_2, vehicle_model);
            // lateral_control = controller.LQRController(robot_state, planning_path_2, vehicle_model);

            // 给 仿真器 的 发送 控制信号（横向控制 和 纵向控制）
            control_msg.acceleration_pct = acceleration_pct;                    // 油门踏板开度
            control_msg.braking_pct = braking_pct;                              // 刹车踏板开度
            control_msg.target_wheel_angle = target_wheel_angle;                // 方向盘转角  
            control_msg.target_wheel_angular_rate = target_wheel_angular_rate;  // 方向盘转速
            

            // 给 仿真器 的 1/10 秒 时间 执行到 预期的控制信号 
            control_pub.publish(control_msg);   // 发布控制信号
            loop_rate.sleep();                  // 确保循环以 10 Hz 的频率运行

            // visualize path
            // rviz_yaw = normalizeAngle2(vehicle_model.psi + M_PI/2); 
            rosviewer.publishPose(vehicle_model.x, vehicle_model.y, vehicle_model.psi);
            current_distance = distance(vehicle_model.x, vehicle_model.y, goal_2_pos[0], goal_2_pos[1]);
            
        }
        ROS_INFO("--- The vehicle is arrived at the goal position 2. with distance is: %.2f", current_distance);
        ros::spin();
    }

}
