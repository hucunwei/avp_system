#pragma once
#include <deque>

#include <Eigen/Geometry>
#include <Eigen/Core>
#include "data_struct.h"

class PoseInterpolation {
 public:
  PoseInterpolation() = default;
  void Push(const TimedPose &pose);
  void TrimBefore(double time);
  double EarliestTime() const;
  bool LookUp(TimedPose &pose) const;

 private:
  std::deque<TimedPose> poses_;
};

