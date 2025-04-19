#pragma once

#include "map.h"
#include "pose_interpolation.h"

class ViewerInterface {
 public:
  // display map
  virtual void displayAvpMap(const TimedPose &pose, const Map &avp_map, const std::vector<Slot> &cur_slots)= 0;
  // display ipm detection result
  void displayIpmDetection(const std::vector<cv::Point2f> &corners,
                           const std::vector<cv::Point2f> &slots_corners,
                           const cv::Mat &ipm_image);

 private:
  cv::Mat drawIPMImg(const std::vector<cv::Point2f> &corners,
                     const std::vector<cv::Point2f> &slots_corners,
                     const cv::Mat &ipm_image);

 protected:
  const bool imshow_; // do cv imshow
  int delay_;
  ViewerInterface(bool imshow, bool autorun);
  ~ViewerInterface();
};