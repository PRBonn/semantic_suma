#include "ScanAccumulator.h"

using namespace rv;
using namespace glow;

ScanAccumulator::ScanAccumulator(uint32_t history_size, uint32_t max_points)
    : vbo_(BufferTarget::ARRAY_BUFFER, BufferUsage::DYNAMIC_DRAW), capacity_(history_size), max_size_(max_points) {
  vbo_.resize(history_size * max_points);  // allocate memory & set size accordingly...
  offsets_.resize(history_size, 0);
  poses_.resize(history_size);
  sizes_.resize(history_size, 0);
  timestamps_.resize(history_size, -1);
  std::fill(timestamps_.begin(), timestamps_.end(), -1);

  offsets_[0] = 0;
  sizes_[0] = 0;
  for (uint32_t i = 1; i < capacity_; ++i) {
    offsets_[i] = offsets_[i - 1] + max_points;
    sizes_[i] = 0;
  }
}

void ScanAccumulator::clear() {
  currentIdx_ = -1;
  size_ = 0;
  std::fill(timestamps_.begin(), timestamps_.end(), -1);
}

uint32_t ScanAccumulator::size() const {
  return size_;
}

uint32_t ScanAccumulator::capacity() const {
  return capacity_;
}

uint32_t ScanAccumulator::offset(uint32_t idx) const {
  int32_t newIdx = currentIdx_ - idx;
  if (newIdx < 0) newIdx = capacity_ + newIdx;

  return offsets_[newIdx];
}

const Eigen::Matrix4f& ScanAccumulator::pose(uint32_t idx) const {
  int32_t newIdx = currentIdx_ - idx;
  if (newIdx < 0) newIdx = capacity_ + newIdx;

  return poses_[newIdx];
}

uint32_t ScanAccumulator::timestamp(uint32_t idx) const {
  int32_t newIdx = currentIdx_ - idx;
  if (newIdx < 0) newIdx = capacity_ + newIdx;

  return timestamps_[newIdx];
}

uint32_t ScanAccumulator::size(uint32_t idx) const {
  int32_t newIdx = currentIdx_ - idx;
  if (newIdx < 0) newIdx = capacity_ + newIdx;

  return sizes_[newIdx];
}

glow::GlBuffer<rv::Point3f>& ScanAccumulator::getVBO() {
  return vbo_;
}

void ScanAccumulator::insert(uint32_t timestamp, const Eigen::Matrix4f& pose, const std::vector<rv::Point3f>& points) {
  currentIdx_ += 1;
  if (uint32_t(currentIdx_) >= capacity_) currentIdx_ = 0;

  timestamps_[currentIdx_] = timestamp;
  poses_[currentIdx_] = pose;
  sizes_[currentIdx_] = std::min(max_size_, (uint32_t)points.size());
  vbo_.replace(offsets_[currentIdx_], &points[0], sizes_[currentIdx_]);

  if (size_ < capacity_) size_ += 1;
}

std::vector<Eigen::Matrix4f>& ScanAccumulator::getPoses() {
  return poses_;
}

const std::vector<Eigen::Matrix4f>& ScanAccumulator::getPoses() const {
  return poses_;
}

std::vector<int32_t>& ScanAccumulator::getTimestamps() {
  return timestamps_;
}
const std::vector<int32_t>& ScanAccumulator::getTimestamps() const {
  return timestamps_;
}
