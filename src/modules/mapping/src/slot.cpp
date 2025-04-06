#include "slot.h"

namespace {
double computeCenterDistance(cv::Vec4i line1, cv::Vec4i line2) {
  cv::Point2f p1 ((line1[0] + line1[2]) / 2.0f,
                  (line1[1] + line1[3]) / 2.0f);
  cv::Point2f p2 ((line2[0] + line2[2]) / 2.0f,
                  (line2[1] + line2[3]) / 2.0f);
  return cv::norm(p1 - p2);
}

cv::Point2f computeIntersect(cv::Vec4i line1, cv::Vec4i line2) {
  int x1 = line1[0], y1 = line1[1], x2 = line1[2], y2 = line1[3];
  int x3 = line2[0], y3 = line2[1], x4 = line2[2], y4 = line2[3];
  float denom = (float)((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
  cv::Point2f intersect( (x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4),
                         (x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4));
  return intersect / denom;
}
} // end of namespace

// 保留边角、细小区域 和骨架
cv::Mat skeletonize(const cv::Mat &img) {
  cv::Mat skel(img.size(), CV_8UC1, cv::Scalar(0));
  cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
  cv::Mat temp, eroded;
  do {
    cv::erode(img, eroded, element);
    cv::dilate(eroded, temp, element);
    cv::subtract(img, temp, temp);  // 细小区域
    cv::bitwise_or(skel, temp, skel);
    eroded.copyTo(img);
  } while (0 != cv::countNonZero(img));
  return skel;
}

void removeIsolatedPixels(cv::Mat &src, int min_neighbors) {
  CV_Assert(src.type() == CV_8UC1);  // 确保单通道灰度图
  cv::Mat dst = src.clone();
  for (int y = 1; y < src.rows - 1; ++y) {
    for (int x = 1; x < src.cols - 1; ++x) {
      if (src.at<uchar>(y, x) > 0) {  // 如果是前景图像
        int neighbor_count = 0;       // 相邻前景像素的计数器
        // 检查8邻域内的像素
        for (int ny = -1; ny <= 1; ++ny) {
          for (int nx = -1; nx <= 1; ++nx) {
            if (ny == 0 && nx == 0) continue;  // 跳过自己
            if (src.at<uchar>(y + ny, x + nx) > 0) {
              ++neighbor_count;  // 增加邻居数
            }
          }
        }
        // 如果相邻前景像素小于阈值，则认为该点是孤立的，并将其去除
        if (neighbor_count < min_neighbors) {
          dst.at<uchar>(y, x) = 0;
        }
      }
    }
  }
  src = dst;
}

bool isSlotShortLine(const cv::Point2f &point1,
                     const cv::Point2f &point2,
                     const cv::Mat &image) {
  cv::LineIterator it(image, point1, point2, 4);
  int positiveIndex = 0;
  const double len = cv::norm(point1 - point2);
  int delta = 10;
  if (std::fabs(len - kShortLinePixelDistance) < delta) {
    for (int i = 0; i < it.count; ++i, ++it) {
      int color = image.at<uchar>(std::round(it.pos().y), std::round(it.pos().x));
      if (color > 0) {
        positiveIndex++;
      }
    }
    if (positiveIndex > kShortLineDetectPixelNumberMin) {
      return true;
    }
  }
  return false;
}

bool isSlotLongLine(const cv::Mat &line_img,
                    const cv::Point2f &start,
                    const cv::Point2f &dir) {
  int cnt{0};
  cv::Point2f pt(start);
  for (int l = 1; l < kLongLinePixelDistance; ++l) {
    pt += dir;
    if (pt.y <= 0 || pt.y >= kIPMImgHeight ||
        pt.x <= 0 || pt.x >= kIPMImgWidth) {
      continue;
    }
    if (line_img.at<uchar>(pt) > 0) {
      ++cnt;
    }
  }
  return cnt > kLongLineDetectPixelNumberMin;
}

std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point2f>> detectSlot(
    const cv::Mat &slot_img, const std::vector<cv::Vec4i> &lines) {

  // estimate the normalized line direction
  auto get_dir = [](const cv::Vec4i& line) -> Eigen::Vector2f{
    // Extract the two endpoints from cv::Vec4i
    int x1 = line[0];
    int y1 = line[1];
    int x2 = line[2];
    int y2 = line[3];
    Eigen::Vector2f coeffs{x2 - x1, y2 - y1};
    coeffs.normalize();
    return coeffs;
  };

  // estimate angle between two intersection lines
  auto get_angle = [&](const cv::Vec4i& line1, const cv::Vec4i& line2) -> float{
      auto cos = get_dir(line1).dot(get_dir(line2));
      auto angle = acos(cos) * kToDeg;
      return angle;
  };

  /////////////// TODO 1 detect corners from lines ///////////////
  std::vector<cv::Point2f> corners; // corners detected from lines
  for (int i = 0; i < lines.size(); ++i) {
    for (int j = i; j < lines.size(); ++j) {
      cv::Point2f pt = computeIntersect(lines[i], lines[j]);
      auto angle_in_deg = get_angle(lines[i], lines[j]);
      auto k = std::round(pt.x);
      auto g = std::round(pt.y);
      if(std::fabs(angle_in_deg - 90) < 10 && slot_img.at<uchar>(k, g) >= 0) { //kSlotGray1
        corners.push_back(pt);
      }
    }
  }

  std::vector<cv::Point2f> merged_corners; // corners detected from lines
  for(int i = 0; i < corners.size(); ++i) {
    CornerPoint corner(Eigen::Vector3d{corners[i].x, corners[i].y, 0});
    for(int j = i; j < corners.size(); ++j) {
      Eigen::Vector3d pt{corners[j].x, corners[j].y, 0};
      corner.absorb(pt, 2); //2 pixel = 0.04
    }
    merged_corners.push_back(cv::Point2f{corner.center()[0], corner.center()[1]});
  }

  /////////////// TODO 2 detect slots in slot_img ///////////////
  std::vector<cv::Point2f> slot_points; // Every 4 consecutive points compose a slot
  for(int i = 0; i < merged_corners.size(); ++i) {
    const cv::Point2f& pti = merged_corners[i];
    for(int j = i; j < merged_corners.size(); ++j) {
      const cv::Point2f& ptj = merged_corners[j];
      if(isSlotShortLine(pti, ptj, slot_img)){
          cv::Point2f dir = pti - ptj;
          float magnitude = std::sqrt(dir.x * dir.x + dir.y * dir.y);
          cv::Point2f normal(-dir.y, dir.x);
          cv::Point2f unit_normal = normal / magnitude;
          if(isSlotLongLine(slot_img, pti, unit_normal) && isSlotLongLine(slot_img, ptj, unit_normal)){
            slot_points.push_back(pti);
            slot_points.push_back(ptj);
            slot_points.push_back(ptj + unit_normal*kLongLinePixelDistance);
            slot_points.push_back(pti + unit_normal*kLongLinePixelDistance);
          }else if(isSlotLongLine(slot_img, pti, -unit_normal) && isSlotLongLine(slot_img, ptj, -unit_normal)){
            slot_points.push_back(pti);
            slot_points.push_back(ptj);
            slot_points.push_back(ptj - unit_normal*kLongLinePixelDistance);
            slot_points.push_back(pti - unit_normal*kLongLinePixelDistance);
          }
      }
    }
  }

  return std::make_tuple(merged_corners, slot_points);
}
