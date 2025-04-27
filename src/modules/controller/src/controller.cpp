//
// Created by ubuntu on 25-3-8.
//
#include "controller.h"

/**
 * find the index of the point on the reference path which is closest to robot_state
 * @param robot_state robot state（x,y）
 * @param refer_path  reference path
 * @return the index of the closest point
 */
#define EPS 1.0e-4

double Controller::calTargetIndex(const vector<double> &robot_state, const vector<vector<double>> &refer_path) {
    vector<double> dists;
    for (vector<double> xy: refer_path) {
        double dist = sqrt(pow(xy[0] - robot_state[0], 2) + pow(xy[1] - robot_state[1], 2));
        dists.push_back(dist);
    }
    return min_element(dists.begin(), dists.end()) - dists.begin(); //返回vector最小元素的下标
}

double Controller::calTargetIndex(vector<double> robot_state, vector<vector<double>> refer_path, double l_d) {
    vector<double> dists;
    for (vector<double> xy: refer_path) {
        double dist = sqrt(pow(xy[0] - robot_state[0], 2) + pow(xy[1] - robot_state[1], 2));
        dists.push_back(dist);
    }
    double min_ind = min_element(dists.begin(), dists.end()) - dists.begin(); //返回vector最小元素的下标

    double delta_l = sqrt(
            pow(refer_path[min_ind][0] - robot_state[0], 2) + pow(refer_path[min_ind][1] - robot_state[1], 2));

    while (l_d > delta_l && min_ind < refer_path.size() - 1) {
        delta_l = sqrt(pow(refer_path[min_ind + 1][0] - robot_state[0], 2) +
                       pow(refer_path[min_ind + 1][1] - robot_state[1], 2));
        min_ind += 1;
    }
    return min_ind;
}

double normalizeAngle(double angle) {
    while (angle > PI) {
        angle -= 2.0 * PI;
    }
    while (angle < -PI) {
        angle += 2.0 * PI;
    }
    return angle;
}

MatrixXd CalRicatti(MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R) {
    MatrixXd Qf = Q;
    MatrixXd P = Qf;
    MatrixXd P_;
    for (int i = 0; i < 100; i++) {
        P_ = Q + A.transpose() * P * A -
             A.transpose() * P * B * (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
        if ((P_ - P).maxCoeff() < EPS && (P - P_).maxCoeff() < EPS)break;
        P = P_;
    }
    return P_;
}



double Controller::PIDController(const vector<double> &robot_state, const vector<vector<double>> &refer_path,
                                 const KinematicModel &ugv) {
    double Kp = 0.5;
    double Ki = 0;
    double Kd = 10;
    static double sum_error = 0;
    static double pre_error = 0;

    double min_ind = calTargetIndex(robot_state, refer_path);
    double alpha = atan2(refer_path[min_ind][1] - robot_state[1], refer_path[min_ind][0] - robot_state[0]);
    double l_d = sqrt(
            pow(refer_path[min_ind][0] - robot_state[0], 2) + pow(refer_path[min_ind][1] - robot_state[1], 2));
    double theta_e = alpha - ugv.psi;
    double e_y = -l_d * sin(theta_e);

    double error = 0 - e_y;
    double u = error * Kp + sum_error * Ki + (error - pre_error) * Kd; 
    
    u = u > PI / 6 ? PI / 6 : u;     // 条件 ? 表达式1 : 表达式2,  上限值：PI / 6
    u = u < -PI / 6 ? -PI / 6 : u;   // 下限值：- PI / 6
    pre_error = error;
    sum_error += error;

    return u;
}

double
Controller::PurePursuitController(const vector<double> &robot_state, const vector<vector<double>> &refer_path,
                                  const KinematicModel &ugv) {

    double l_d = 3; // 前视距离
    double L = 2; // 轴距
    // Your code
    // 明确 目标点 target
    double target_ind = calTargetIndex(robot_state, refer_path,l_d);
    double target_x = refer_path[target_ind][0], target_y = refer_path[target_ind][1];

    // 计算车辆当前位置与目标点之间的角度 alpha
    double alpha = atan2(target_y - robot_state[1], target_x - robot_state[0] ) - robot_state[2] ; // psi = robot_state[2] 是车辆当前航向角
    alpha = normalizeAngle(alpha); // 归一化角度 alpha 到 [-π, π]

    // 根据 Pure Pursuit 公式计算转向角 delta
    double delta = atan2(2.0 * L * sin(alpha), l_d);

    // 对转向角进行限幅，防止过大或过小
    delta = delta > PI / 4 ? PI / 4 : delta;  // 上限值：PI / 4
    delta = delta < -PI / 4 ? -PI / 4 : delta; // 下限值：-PI / 4

    return delta;
}



double Controller::StanlyController(const vector<double> &robot_state, const vector<vector<double>> &refer_path,
                                    const KinematicModel &ugv) {
    // Your code
    double d = 10 * ugv.v; //  d is a parameter that is propotional to the vehicle speed v
    // front wheel info
    vector<double> robot_front_state(4);
    robot_front_state[0] = robot_state[0] + ugv.L * cos(robot_state[2]);      // front_x = ugv.x + ugv.L * cos(ugv.psi); 
    robot_front_state[1] = robot_state[1] + ugv.L * sin(robot_state[2]);      // front_y = ugv.y + ugv.L * sin(ugv.psi); 
    robot_front_state[2] = robot_state[2];                                    // front_psi = ugv.psi; 
    robot_front_state[3] = robot_state[3];                                    // front_v = ugv.v; 


    double min_ind = calTargetIndex(robot_front_state, refer_path);
    double e_y = sqrt( pow(refer_path[min_ind][0] - robot_front_state[0], 2) + pow(refer_path[min_ind][1] - robot_front_state[1], 2) );
    double error_lateral = atan2(e_y, d); // arctan( e_y / d )

    double psi_ref_heading = refer_path[min_ind][2];
    double psi_front_heading = robot_front_state[2];
    double error_heading = psi_ref_heading -  psi_front_heading;
    
    
    double u = error_lateral + error_heading;
    // double u = error_lateral;
    u = normalizeAngle(u);
    cout << "error_lateral = " << error_lateral << "( " << error_lateral * 180 / M_PI  << " ),   " << 
            "error_heading = " << error_heading << "( " << error_heading * 180 / M_PI  << " ),   " << 
            "u = " << u << "( " << u * 180 / M_PI  << " ),   " << endl;
    // u = u > PI / 6 ? PI / 6 : u;     // 条件 ? 表达式1 : 表达式2,  上限值：PI / 6
    // u = u < -PI / 6 ? -PI / 6 : u;   // 下限值：- PI / 6
    u = u > PI / 4 ? PI / 4 : u;     // 上限值：PI / 4
    u = u < -PI / 4 ? -PI / 4 : u;   // 下限值：- PI / 4
    return u;
}

double Controller::LQRController(const vector<double> &robot_state, const vector<vector<double>> &refer_path,
                                 const KinematicModel &ugv) {

    // 找到参考路径上最近的目标点索引
    double ref_index = calTargetIndex(robot_state, refer_path);
    // 获取目标点的状态 (x_ref, y_ref, psi_ref)
    double x_ref = refer_path[ref_index][0], y_ref = refer_path[ref_index][1], psi_ref = refer_path[ref_index][2];
    double v_ref = 2.0, delta_ref = 0;
    VectorXd control_ref(2);
    control_ref << v_ref, delta_ref;

    // 当前车辆状态 (x, y, psi, v)
    double x_current = robot_state[0], y_current = robot_state[1], psi_current = robot_state[2];
    double v_current = robot_state[3], delta_current = 0;
    
    // 计算状态误差：e_x, e_y, e_psi; 和 控制误差： e_v, e_delta
    double error_x = x_current - x_ref;
    double error_y = y_current - y_ref;
    double error_psi    = normalizeAngle(psi_current - psi_ref); // 归一化角度
    double error_v      = v_current   - v_ref;
    double error_delta  = delta_current - delta_ref;

    // 误差状态 向量 [error_x, error_y, error_psi]
    VectorXd state_error(3);
    state_error << error_x, error_y, error_psi;
    // 误差控制 向量[error_v, error_delta]
    VectorXd control_error(2);
    control_error << error_v, error_delta;

    // 离散状态空间矩阵 A 和 B
    MatrixXd A, B;
    vector <MatrixXd> system_model = ugv.stateSpace(delta_ref, psi_ref);
    A = system_model[0];
    B = system_model[1];

    // 定义权重矩阵 Q 和 R for Quadratic cost functiion J
    MatrixXd Q(3,3), R(2,2);
    Q << 1.0, 0.0, 0.0,  // 对 error_x 的权重
         0.0, 1.0, 0.0,  // 对 error_y 的权重
         0.0, 0.0, 1.0;  // 对 error_psi 的权重
    R << 0.001, 0.0,       // 对速度控制的权重
         0.0, 1.0;       // 对转向角控制的权重
    
    Q = 20 * Q;

    // 计算 Riccati 方程的解 P
    MatrixXd P = CalRicatti(A, B, Q, R);
    // 计算反馈增益矩阵 K
    MatrixXd K = -(B.transpose() * P * B + R).inverse() * B.transpose() * P * A;
    // 最优 误差控制 u = -K * state_error
    MatrixXd error_control_optimal = -K * state_error;
    // 最优 控制输入
    MatrixXd control_optimal = error_control_optimal + control_ref;
    // 提取横向控制输入 delta_f (转向角)
    double delta_f = normalizeAngle(control_optimal(1));

    cout << "v_optimal = " << control_optimal(0) <<  ",    " << 
            "delta_f = " << delta_f << "( " << delta_f * 180 / M_PI  << " ),   " << endl;

    delta_f = delta_f > PI / 4 ? PI / 4 : delta_f;     // 上限值：PI / 4
    delta_f = delta_f < -PI / 4 ? -PI / 4 : delta_f;   // 下限值：- PI / 4
    // delta_f = delta_f > PI / 6 ? PI / 6 : delta_f;     // 上限值：PI / 4
    // delta_f = delta_f < -PI / 6 ? -PI / 6 : delta_f;   // 下限值：- PI / 4

    return delta_f;
}