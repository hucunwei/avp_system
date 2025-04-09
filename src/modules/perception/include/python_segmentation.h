#ifndef PYTHON_SEGMENTATION_H
#define PYTHON_SEGMENTATION_H

#include <opencv2/opencv.hpp>
#include <string>
#include <memory>

class PythonSegmentation {
public:
  PythonSegmentation(const std::string& config_path, const std::string& weight_path);
  ~PythonSegmentation();

  cv::Mat process(const cv::Mat& input);

  // Non-copyable
  PythonSegmentation(const PythonSegmentation&) = delete;
  PythonSegmentation& operator=(const PythonSegmentation&) = delete;

private:
  struct Impl;
  std::unique_ptr<Impl> pImpl;
};

#endif