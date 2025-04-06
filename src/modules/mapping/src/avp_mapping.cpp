#include <cmath>
#include <opencv2/opencv.hpp>
//#include <opencv2/ximgproc.hpp>
#include <vector>

#include "avp_mapping.h"

AvpMapping::AvpMapping() {
  pre_key_pose_.time_ = -1;
  T_vehicle_ipm_.linear().setIdentity();
  T_vehicle_ipm_.translation() = Eigen::Vector3d(0.0, 1.32, 0.0);
}

bool AvpMapping::isKeyFrame(const TimedPose &pose) {
  if ((pose.t_ - pre_key_pose_.t_).norm() > 0.1 ||
      std::fabs(GetYaw(pre_key_pose_.R_.inverse() * pose.R_)) > 5. * kToRad ||
      pose.time_ - pre_key_pose_.time_ > 30 || pre_key_pose_.time_ < -1) {
    pre_key_pose_ = pose;
    return true;
  }
  return false;
}

void AvpMapping::processPose(const TimedPose &pose) {
  pose_interpolation_.Push(pose);
}

void AvpMapping::processImage(double time, const cv::Mat &ipm_seg_img) {
  if (time < pose_interpolation_.EarliestTime()) {
    return;
  }
  TimedPose pose;
  pose.time_ = time;
  if (pose_interpolation_.LookUp(pose) && isKeyFrame(pose)) {
    pose_interpolation_.TrimBefore(time);
    std::cout << "keyframe : " << "  t = " << pose.t_.transpose().head(2)
              << ", yaw = " << GetYaw(pose.R_) * kToDeg << std::endl;

    cv::Mat img_gray;
    cv::cvtColor(ipm_seg_img, img_gray, cv::COLOR_BGR2GRAY);
    extractSlot(img_gray, pose);
  }
}
Eigen::Vector3d AvpMapping::ipmPlane2Global(const TimedPose &T_world_vehicle, const cv::Point2f &ipm_point){
  Eigen::Vector3d pt_global;
  //////////////////////// TODO: transform ipm pixel to point in global ///////////////////////
  Eigen::Vector3d pt_vehicle_center;
  pt_vehicle_center.setZero();
  pt_vehicle_center(0) = -(kIPMImgWidth * 0.5 - ipm_point.x) * kPixelScale;
  pt_vehicle_center(1) = (kIPMImgHeight * 0.5 - ipm_point.y) * kPixelScale;
  Eigen::Vector3d pt_vehicle = T_vehicle_ipm_ * pt_vehicle_center;
  pt_global = T_world_vehicle.R_ * pt_vehicle + T_world_vehicle.t_;
  return pt_global;
}


void AvpMapping::extractSlot(const cv::Mat &img_gray, const TimedPose &T_world_vehicle) {
  cv::Mat slot_img = cv::Mat::zeros(img_gray.size(), img_gray.type());
  for (int i = 0; i < img_gray.rows; ++i) {
    for (int j = 0; j < img_gray.cols; ++j) {
      if (kSlotGray == img_gray.at<uchar>(i, j) || kSlotGray1 == img_gray.at<uchar>(i, j)) {
        slot_img.at<uchar>(i, j) = 254;
      } else if(kArrowGray == img_gray.at<uchar>(i, j)) {
        avp_map_.addSemanticElement(SemanticLabel::kArrowLine,
                                    {ipmPlane2Global(T_world_vehicle, cv::Point2f(j, i))});
      } else if(kDashGray == img_gray.at<uchar>(i, j)) {
        avp_map_.addSemanticElement(SemanticLabel::kDashLine,
                                    {ipmPlane2Global(T_world_vehicle, cv::Point2f(j, i))});
      } else{}
    }
  }


  int kernelSize = 15;
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,
                                             cv::Size(kernelSize, kernelSize));
  cv::Mat closed;
  cv::morphologyEx(slot_img, closed, cv::MORPH_CLOSE, kernel); // cv::Point(-1, -1), 2);
  auto line_img = closed.clone();

  cv::Mat skel = skeletonize(closed);
  removeIsolatedPixels(skel, 1);
  // extract slot lines
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(skel, lines, 1, CV_PI / 180, 50, 50, 50); //100, 50, 5
  // detect slots
  auto [corners, slot_points] = detectSlot(slot_img, lines);

  // add detected slots to map
  std::vector<Slot> slots(slot_points.size() / 4);
  int index = 0;
  for (auto &slot : slots){
    slot.corners_[0] = ipmPlane2Global(T_world_vehicle, slot_points[index++]);
    slot.corners_[1] = ipmPlane2Global(T_world_vehicle, slot_points[index++]);
    slot.corners_[2] = ipmPlane2Global(T_world_vehicle, slot_points[index++]);
    slot.corners_[3] = ipmPlane2Global(T_world_vehicle, slot_points[index++]);
    avp_map_.addSlot(slot); // add slot to map
  }
  // update viewer
  if (viewer_){
    viewer_->displayAvpMap(T_world_vehicle, avp_map_, slots);
    viewer_->displayIpmDetection(corners, slot_points, img_gray);
  }
}


