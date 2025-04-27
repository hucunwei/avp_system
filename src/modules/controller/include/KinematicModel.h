
#ifndef KINEMATICMODEL_H
#define KINEMATICMODEL_H

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class KinematicModel {
public:
    double x, y, psi, v, L, dt;
public:
    KinematicModel();

    KinematicModel(double x, double y, double psi, double v, double l, double dt);

    vector<double> getState();

    void updateState(double velocity, double delta_f);

    vector<MatrixXd> stateSpace(double ref_delta, double ref_yaw) const;

};

class VehicleChassis {
public:
    double speed_mps;     // 速度_m/s 状态
    double selected_gear; // 0:空档, 1:行使, 2:倒车, 3:泊车(P挡)， 4:低速
    double throttle_pct ; // 0 to 1 油门踏板 开度 状态
    double brake_pct;     // 0 to 1 刹车踏板 开度 状态
    double steer_pct;     // -1 to 1 方向盘 转角 状态 
    double steer_angle;   // -1 to 1 方向盘 转角 状态 --> - pi/4 to pi/4
    VehicleChassis(double speed_mps, double selected_gear, double throttle_pct, double brake_pct, double steer_pct, double steer_angle) :
            speed_mps(speed_mps), selected_gear(selected_gear), 
            throttle_pct(throttle_pct), brake_pct(brake_pct),
            steer_pct(steer_pct), steer_angle(steer_angle) {};
};

#endif
