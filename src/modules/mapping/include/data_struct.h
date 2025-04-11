#pragma once

#include <Eigen/Geometry>

struct TimedPose {
  double time_;
  Eigen::Vector3d t_;
  Eigen::Quaterniond R_;
};

struct WheelMeasurement {
  double time_;
  double velocity_;
  double yaw_rate_;
};

template <typename T>
T GetYaw(const Eigen::Quaternion<T> &rotation) {
  const Eigen::Matrix<T, 3, 1> direction =
      rotation * Eigen::Matrix<T, 3, 1>::UnitX();
  return atan2(direction.y(), direction.x());
}