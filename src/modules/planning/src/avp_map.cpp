#include "avp_map.h"


std::vector<Slot> AvpMap::getAllSlots() const {
  std::vector<Slot> all_slots(slots_.size());
  for (int i = 0; i < slots_.size(); ++i) {
    Eigen::Vector3d slot_center(0.0, 0.0, 0.0);                           // 计算车位的中心点
    for (int j = 0; j < 4; ++j) {
      all_slots[i].corners_[j] = corner_points_[slots_[i][j]].center();   // 存储车位的 角点
      slot_center += all_slots[i].corners_[j];
    }
    all_slots[i].slot_center = slot_center /= 4.0;                        // 存储 车位的中心点
    all_slots[i].slot_center.z() = 0;                                     // 修正 z 为 0
    Eigen::Vector3d slot_direction = all_slots[i].corners_[1] - all_slots[i].corners_[2];  // 计算 车位的方向向量
    all_slots[i].yaw = atan2(slot_direction.y(), slot_direction.x()) ;
    all_slots[i].updateBoundingBox2();                                     // 计算 车位的 bounding box, 方便后续 2d Nav Goal 判断是哪个车位
  }
  return std::move(all_slots);
}


void AvpMap::updatePureSlots() {
  pure_slots_ = this->getAllSlots();  // 使用 this 指针明确作用域
}

const std::unordered_set<Eigen::Vector3i, Vector3iHash> &
AvpMap::getSemanticElement(SemanticLabel label) const {
  return semantic_grid_map_.at(label);
}

// add semantic element
void AvpMap::addSemanticElement(SemanticLabel label, const Eigen::Vector3d &pt) {
  Eigen::Vector3i grid_pt;
  grid_pt.x() = std::round(pt.x() * kPixelScaleInv);
  grid_pt.y() = std::round(pt.y() * kPixelScaleInv);
  grid_pt.z() = std::round(pt.z() * kPixelScaleInv);
  semantic_grid_map_[label].insert(grid_pt);  // like a voxel filter
}

// add slot to map
void AvpMap::addSlot(const Slot &new_slot) {
  // add slot corner to map
  SlotIndex new_slot_index;
  Eigen::Vector3d new_slot_center = Eigen::Vector3d::Zero();
  for (int i = 0; i < 4; ++i) {
    new_slot_index[i] = addSlotCorner(new_slot.corners_[i]);
    new_slot_center += new_slot.corners_[i];
  }
  new_slot_center *= 0.25;
  // add to map if new slot
  bool isNewSlot = true;
  Eigen::Vector3d slot_center;
  for (const auto slot_index : slots_) {
    slot_center = Eigen::Vector3d::Zero();
    for (int j = 0; j < 4; ++j) {
      slot_center += corner_points_[slot_index[j]].center();
    }
    slot_center *= 0.25;
    if ((slot_center - new_slot_center).norm() < 1.0) {
      isNewSlot = false;
      break;
    }
  }
  if (isNewSlot) {
    slots_.push_back(new_slot_index);
  }
}

size_t AvpMap::addSlotCorner(const Eigen::Vector3d &pt) {
  size_t index = corner_points_.size();
  for (int i = 0; i < corner_points_.size(); ++i) {
    if (corner_points_[i].absorb(pt, kNeighborDistance * kPixelScale)) {
      index = i;
      break;
    }
  }
  if (corner_points_.size() == index) {
    corner_points_.emplace_back(pt);
    bounding_box_.extend(pt);
  }
  return index;
}

void AvpMap::save(const std::string &filename) const {
  std::ofstream ofs(filename, std::ios::binary);
  if (!ofs.is_open()) {
    std::cerr << "Unable to open map file: " << filename << std::endl;
    return;
  }

  io::saveVector(ofs, corner_points_);
  io::saveVector(ofs, slots_);
  std::cout << "save corners: " << corner_points_.size() << std::endl;
  std::cout << "save slots: " << slots_.size() << std::endl;

  std::vector<Eigen::Vector3i> semantic_grid;
  for (int i = 0; i < SemanticLabel::kTotalLabelNum; ++i) {
    semantic_grid.assign(semantic_grid_map_[i].begin(),
                         semantic_grid_map_[i].end());
    io::saveVector(ofs, semantic_grid);
    std::cout << "save label_" << i << ": " << semantic_grid.size()
              << std::endl;
  }
  ofs.close();
  std::cout << "avp map saved to " << filename << std::endl;
}

void AvpMap::load(const std::string &filename) {
  if (!corner_points_.empty()) {
    std::cerr << "map have already loaded, fail to load map file: " << filename
              << std::endl;
    return;
  }
  std::ifstream ifs(filename, std::ios::binary);
  if (!ifs.is_open()) {
    std::cerr << "Unable to load map file: " << filename << std::endl;
    return;
  }
  io::loadVector(ifs, corner_points_);
  io::loadVector(ifs, slots_);
  for (const auto &corner : corner_points_) {
    bounding_box_.extend(corner.center());  // extend bounding box
  }
  std::cout << "load corners: " << corner_points_.size() << std::endl;
  std::cout << "load slots: " << slots_.size() << std::endl;

  std::vector<Eigen::Vector3i> semantic_grid;
  for (int i = 0; i < SemanticLabel::kTotalLabelNum; ++i) {
    io::loadVector(ifs, semantic_grid);
    semantic_grid_map_[i].insert(semantic_grid.begin(), semantic_grid.end());
    std::cout << "load label_" << i << ": " << semantic_grid.size()
              << std::endl;
  }
  ifs.close();
  if (semantic_grid_map_[kSlot].empty()) {
    updatePureSlots(); // udpate pure_slots
    discretizeLine(pure_slots_);
    // discretizeLine(getAllSlots());

  }

  std::cout << "avp map loaded from " << filename << std::endl;
}

void interpolatePoints(std::vector<Eigen::Vector3d> &points) {
  if (points.size() < 2)
    points = {};

  std::vector<Eigen::Vector3d> result;
  for (size_t i = 0; i < points.size(); ++i) {
    if (i == 0)
      continue;
    const Eigen::Vector3d &p1 = points[i];
    const Eigen::Vector3d &p2 = points[(i + 1) % 4];

    result.push_back(p1);
    int insertionsPerSegment =
        std::max(1, static_cast<int>((p1 - p2).norm() / 0.05));
    for (int j = 1; j <= insertionsPerSegment; ++j) {
      double t = static_cast<double>(j) / (insertionsPerSegment + 1);
      double nx = p1(0) + t * (p2(0) - p1(0));
      double ny = p1(1) + t * (p2(1) - p1(1));
      double nz = p1(2) + t * (p2(2) - p1(2));
      result.emplace_back(nx, ny, nz);
    }
  }

  result.push_back(points.back());
  result.swap(points);
}

void AvpMap::discretizeLine(const std::vector<Slot> &slots) {
// void AvpMap::discretizeLine(std::vector<Slot> &slots) {
  // pure_slots_ = slots;
  for (const auto &slot : slots) {
    std::vector<Eigen::Vector3d> single_slot;
    for (int j = 0; j < 4; ++j) {
      single_slot.push_back(slot.corners_[j]);
    }
    interpolatePoints(single_slot);
    for (const auto &pt : single_slot) {
      addSemanticElement(SemanticLabel::kSlot, pt);
    }
  }
  std::cout << "kSlot discretize points: "
            << getSemanticElement(SemanticLabel::kSlot).size() << std::endl;
}