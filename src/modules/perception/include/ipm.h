#pragma once
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
// #include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <unordered_map>

struct MEICamera {
  MEICamera(const std::string &intrinsicFile, const std::string &extrinsicFile);
  ~MEICamera() = default;

  void DebugString() const;

  bool is_valid_{false};
  std::string id_;
  int height_, width_;
  double xi_;
  cv::Mat K_, D_;
  Eigen::Affine3d T_vehicle_cam_; // Extrinsic: from camera to vehicle
};

// refer to google Openchisel project.
// Spatial hashing function from Matthias Teschner's paper:
// Optimized Spatial Hashing for Collision Detection of Deformable Objects
using PixelIndex = Eigen::Vector3i;
struct PixelIndexHasher {
  // Three large primes are used for spatial hashing.
  static constexpr size_t p1 = 73856093;
  static constexpr size_t p2 = 19349663;
  static constexpr size_t p3 = 83492791;
  std::size_t operator()(const PixelIndex& key) const {
    return (key(0) * p1 ^ key(1) * p2 ^ key(2) * p3);
  }
};

struct PixelId {
   int x, y;
};

using IPMToImagMap = std::unordered_map<PixelIndex, PixelId, PixelIndexHasher>;

class IPM {
 public:
  IPM();
  ~IPM() = default;

  void AddCamera(const std::string &intrinsicsFile,
                 const std::string &extrinsicFile);
  void CreateIPMToImageMap();
  cv::Mat GenerateIPMImage(const std::vector<cv::Mat> &images) const;

 private:
  std::vector<MEICamera> cameras_;

  int ipm_img_h_ = 1000;
  int ipm_img_w_ = 1000;
  const double pixel_scale_ = 0.02;
  const double ipm_img_height_ = 0.0;

  IPMToImagMap ipm_image_map_;
};