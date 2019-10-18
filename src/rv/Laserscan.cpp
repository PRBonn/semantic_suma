#include "rv/Laserscan.h"

namespace rv
{

Laserscan::Laserscan()
{

}

Laserscan::Laserscan(const Transform& pose, const std::list<Point3f>& points, const std::list<float>& remissions) :
    mPose(pose)
{
  points_.assign(points.begin(), points.end());
  remissions_.assign(remissions.begin(), remissions.end());
}

Laserscan::Laserscan(const Transform& pose, const std::list<Point3f>& points, const std::list<float>& remissions, const std::list<Point3f>& labels) :
    mPose(pose)
{
  points_.assign(points.begin(), points.end());
  remissions_.assign(remissions.begin(), remissions.end());
  labels_.assign(labels.begin(), labels.end());
}

/** \brief clear **/
void Laserscan::clear()
{
  points_.clear();
  remissions_.clear();
  normals_.clear();

  labels_.clear(); // for semantic map
}

/** \brief getter for points/normals/remission **/
const Point3f& Laserscan::point(uint32_t i) const
{
  assert(i < points_.size());
  return points_[i];
}

float Laserscan::remission(uint32_t i) const
{
  assert(i < remissions_.size());
  return remissions_[i];
}

const Normal3f& Laserscan::normal(uint32_t i) const
{
  assert(i < normals_.size());
  return normals_[i];
}

const Point3f& Laserscan::label(uint32_t i) const
{
  assert(i < labels_.size());
  return labels_[i];
}


Transform& Laserscan::pose()
{
  return mPose;
}

const Transform& Laserscan::pose() const
{
  return mPose;
}

uint32_t Laserscan::size() const
{
  return points_.size();
}

std::vector<Point3f>& Laserscan::points()
{
  return points_;
}

const std::vector<Point3f>& Laserscan::points() const
{
  return points_;
}

std::vector<float>& Laserscan::remissions()
{
  return remissions_;
}

const std::vector<float>& Laserscan::remissions() const
{
  return remissions_;
}

const std::vector<Normal3f>& Laserscan::normals() const
{
  return normals_;
}

std::vector<Normal3f>& Laserscan::normals()
{
  return normals_;
}

std::vector<Point3f>& Laserscan::labels()
{
  return labels_;
}

const std::vector<Point3f>& Laserscan::labels() const
{
  return labels_;
}

bool Laserscan::hasRemission() const
{
  return (points_.size() == remissions_.size());
}

bool Laserscan::hasNormals() const
{
  return (points_.size() == remissions_.size());
}

bool Laserscan::hasLabels() const
{
  return (points_.size() == labels_.size());
}

}
