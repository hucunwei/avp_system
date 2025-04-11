#pragma once
#include <functional>
#include <mutex>
#include <thread>

#include "map.h"
#include "frame.h"

class MapViewer {
 public:
  ~MapViewer();
  MapViewer(const Map &avp_map);

  void showFrame(const Frame &frame) ;

 private:
  void drawGrid(const cv::Mat &map,
                const std::unordered_set<Eigen::Vector3i, Vector3iHash> &grid,
                cv::Vec3b color) const ;

  void run(int width, int height) ;

  std::mutex mutex_;
  int count_ = 1;
  Frame frame_;
  std::thread thread_;

  cv::Mat map_;
  Eigen::AlignedBox3d bounding_box_;
  cv::Point2f offset_;
  std::vector<cv::Point2f> trajectory_;
  std::function<cv::Point2f(const Eigen::Vector3d &)> to_map_pixel_;
};