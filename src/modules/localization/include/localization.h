#pragma once
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

#include "ekf.h"
#include "frame.h"
#include "map.h"
#include "map_viewer/map_viewer.h"
#include "data_struct.h"

class AvpLocalization {
 public:
  AvpLocalization(std::string map_file);

  ~AvpLocalization() = default;

  void processWheelMeasurement(const WheelMeasurement &wheel_measurement);

  void processImage(double time, const cv::Mat &ipm_seg_img);

  void initState(double time, double x, double y, double yaw);

  Eigen::Affine3d imageRegistration();

  const Map &getMap() const { return avp_map_; }

  TimedPose getPose(){
      auto state = ekf_.getState(); // updated pose
      pose_.time_ = state.time;
      pose_.t_ = {state.x, state.y, 0.0};
      pose_.R_ = Eigen::Quaterniond(
          Eigen::AngleAxisd{state.yaw, Eigen::Vector3d::UnitZ()});
    return pose_;
  }
  //  void setViewer(std::shared_ptr<ViewerInterface> viewer) { viewer_ =
  //  viewer; }

 private:
  bool isKeyFrame(const TimedPose &pose);

  Eigen::Vector3d ipmPlane2Global(const TimedPose &T_world_vehicle,
                                  const cv::Point2f &ipm_point);

  void extractFeature(const cv::Mat &img_gray,
                      const TimedPose &T_world_vehicle);

  // 临时先用pcl的kdtree，后续有时间可以换成直接依赖FLANN
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_dash_line_;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_arrow_line_;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_slot_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr dash_line_cloud_in_{
      new pcl::PointCloud<pcl::PointXYZ>};
  pcl::PointCloud<pcl::PointXYZ>::Ptr arrow_line_cloud_in_{
      new pcl::PointCloud<pcl::PointXYZ>};
  pcl::PointCloud<pcl::PointXYZ>::Ptr slot_cloud_in_{
      new pcl::PointCloud<pcl::PointXYZ>};

  TimedPose pre_key_pose_;  // Pose of last key frame
  std::deque<WheelMeasurement> wheel_measurements_;

  Eigen::Affine3d T_vehicle_ipm_;
  Frame curr_frame_;
  Map avp_map_;  // avp map data
  EKF ekf_;      // EKF state estimation

  std::shared_ptr<MapViewer> map_viewer_;
  // std::shared_ptr<ViewerInterface> viewer_{nullptr}; // viewer for debug
  TimedPose pose_;
};
