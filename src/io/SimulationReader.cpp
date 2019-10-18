#include "SimulationReader.h"

#include <glow/glutil.h>
#include <cmath>

using namespace rv;
using namespace glow;

SimulationReader::Ray::Ray(const Eigen::Vector4f& origin, const Eigen::Vector4f& dir) : o(origin), d(dir) {}

Eigen::Vector4f SimulationReader::Ray::operator()(float t) const {
  return o + t * d;
}

SimulationReader::Triangle::Triangle(const Eigen::Vector4f& p1, const Eigen::Vector4f& p2, const Eigen::Vector4f& p3) {
  vertices[0] = p1;
  vertices[1] = p2;
  vertices[2] = p3;
}

void SimulationReader::addCube(const Eigen::Matrix4f& pose, float size) {
  std::vector<Eigen::Vector4f> c;
  c.push_back(pose * Eigen::Vector4f(0.5f * size, 0.5f * size, -0.5f * size, 1));
  c.push_back(pose * Eigen::Vector4f(-0.5f * size, 0.5f * size, -0.5f * size, 1));
  c.push_back(pose * Eigen::Vector4f(-0.5f * size, -0.5f * size, -0.5f * size, 1));
  c.push_back(pose * Eigen::Vector4f(0.5f * size, -0.5f * size, -0.5f * size, 1));
  c.push_back(pose * Eigen::Vector4f(0.5f * size, 0.5f * size, 0.5f * size, 1));
  c.push_back(pose * Eigen::Vector4f(-0.5f * size, 0.5f * size, 0.5f * size, 1));
  c.push_back(pose * Eigen::Vector4f(-0.5f * size, -0.5f * size, 0.5f * size, 1));
  c.push_back(pose * Eigen::Vector4f(0.5f * size, -0.5f * size, 0.5f * size, 1));

  // face 0
  world_.push_back(Triangle(c[0], c[3], c[1]));
  world_.push_back(Triangle(c[1], c[3], c[2]));

  // face 1
  world_.push_back(Triangle(c[2], c[3], c[7]));
  world_.push_back(Triangle(c[2], c[7], c[6]));

  // face 2
  world_.push_back(Triangle(c[0], c[4], c[7]));
  world_.push_back(Triangle(c[0], c[7], c[3]));

  // face 3
  world_.push_back(Triangle(c[0], c[5], c[4]));
  world_.push_back(Triangle(c[0], c[1], c[5]));

  // face 4
  world_.push_back(Triangle(c[1], c[2], c[5]));
  world_.push_back(Triangle(c[2], c[6], c[5]));

  // face 5
  world_.push_back(Triangle(c[5], c[6], c[4]));
  world_.push_back(Triangle(c[6], c[7], c[4]));
}

void SimulationReader::addPlane(const Eigen::Matrix4f& pose, float size) {
  std::vector<Eigen::Vector4f> c;
  c.push_back(pose * Eigen::Vector4f(0.5f * size, 0.5f * size, 0, 1));
  c.push_back(pose * Eigen::Vector4f(-0.5f * size, 0.5f * size, 0, 1));
  c.push_back(pose * Eigen::Vector4f(-0.5f * size, -0.5f * size, 0, 1));
  c.push_back(pose * Eigen::Vector4f(0.5f * size, -0.5f * size, 0, 1));

  world_.push_back(Triangle(c[0], c[1], c[2]));
  world_.push_back(Triangle(c[0], c[2], c[3]));
}

SimulationReader::SimulationReader(const std::string& filename) : rng_(1337) {
  // generate dummy world and trajectory.
  addPlane(Eigen::Matrix4f::Identity(), 1000.0f);

  addCube(glTranslate(10, 10, 0.5), 1.0f);
  addCube(glTranslate(112, -15, 1.25) * glRotateZ(Radians(45.0f)), 2.5f);
  addCube(glTranslate(34, 20, 0.75), 1.5f);
  addCube(glTranslate(50, -10, 0.75) * glRotateZ(Radians(79.0f)), 1.5f);
  addCube(glTranslate(65, 5, 0.75) * glRotateZ(Radians(45.0f)), 1.5f);
  addCube(glTranslate(70, -15, 0.85) * glRotateZ(Radians(25.0f)), 1.5f);
  addCube(glTranslate(100, 30, 0.65) * glRotateZ(Radians(0.0f)), 1.5f);
  addCube(glTranslate(120, -10, 0.65) * glRotateZ(Radians(25.0f)), 1.5f);

  addCube(glTranslate(170, -10, 0.65) * glRotateZ(Radians(15.0f)), 1.5f);
  addCube(glTranslate(190, -30, 0.65) * glRotateZ(Radians(35.0f)), 1.5f);
  addCube(glTranslate(230, 15, 0.65) * glRotateZ(Radians(5.0f)), 3.5f);
  addCube(glTranslate(270, -7, 0.65) * glRotateZ(Radians(-3.5f)), 2.5f);
  addCube(glTranslate(280, 20, 0.65) * glRotateZ(Radians(2.0f)), 4.5f);

  addCube(glTranslate(320, 20, 0.65) * glRotateZ(Radians(2.0f)), 4.5f);
  addCube(glTranslate(370, 10, 0.65) * glRotateZ(Radians(15.0f)), 1.5f);
  addCube(glTranslate(390, -30, 0.65) * glRotateZ(Radians(35.0f)), 1.5f);
  addCube(glTranslate(430, -15, 0.65) * glRotateZ(Radians(5.0f)), 3.5f);
  addCube(glTranslate(470, 7, 0.65) * glRotateZ(Radians(-3.5f)), 2.5f);
  addCube(glTranslate(480, 20, 0.65) * glRotateZ(Radians(2.0f)), 4.5f);

  addCube(glTranslate(40, -20, 0.65) * glRotateZ(Radians(15.0f)), 3.0f);

  addCube(glTranslate(50, -75, 5) * glRotateZ(Radians(25.0f)), 10.0f);

  addCube(glTranslate(-20, -54, 0.65) * glRotateZ(Radians(15.0f)) * glRotateX(Radians(2.0f)), 4.5f);

  // Version 2 of HDL-64 E has this beam pattern (see specification)
  float beam_angles[64];
  for (int32_t i = 0; i < 32; ++i) beam_angles[i] = Radians(90.0f - 2.0f + i / 3.0f);
  for (int32_t i = 0; i < 32; ++i) beam_angles[32 + i] = Radians(90.0f + 8.83f + i / 2.0f);

  // precompute all beams.
  beams_.reserve(4 * 360 * 64);
  for (uint32_t i = 0; i < 4 * 360; ++i) {
    float theta = Radians(float(i) / 4.0);
    for (uint32_t j = 0; j < 64; ++j) {
      Eigen::Vector4f beam;
      beam[0] = std::cos(theta) * std::sin(beam_angles[j]);
      beam[1] = std::sin(theta) * std::sin(beam_angles[j]);
      beam[2] = std::cos(beam_angles[j]);
      beam[3] = j;

      beams_.push_back(beam);
    }
  }

  for (uint32_t i = 0; i < 64; ++i) calibration_.push_back(1.0f);

  //  calibration_[30] = 0.94f;
  //  calibration_[34] = 1.03f;
  //
  //  calibration_[57] = 1.02f;
  //  calibration_[61] = 0.94f;
  //  calibration_[63] = 1.05f;

  extrinsicPose_ = Eigen::Matrix4f::Identity();

  Eigen::Matrix4f T = glTranslate(0, 0, 2);
  trajectory_.push_back(T * extrinsicPose_);

  for (uint32_t i = 0; i < 1000; ++i) {
    T = T * glTranslate(1.75 + sigma_speed_ * rng_.getGaussianFloat(), 0, 0);
    trajectory_.push_back(T * glRotateY(0.01 * rng_.getGaussianFloat()));
  }

  for (uint32_t i = 0; i < 30; ++i) {
    T = T * glRotateZ(Radians(-3.0)) * glTranslate(0.5, 0, 0);
    trajectory_.push_back(T * extrinsicPose_);
  }

  float length = 15.0f;
  for (uint32_t i = 0; i < length; ++i) {
    float speed = (length - i) / length * 0.75 + i / length * 1.2;
    T = T * glTranslate(speed, 0, 0);
    trajectory_.push_back(T * extrinsicPose_);
  }

  length = 15.0f;
  for (uint32_t i = 0; i < length; ++i) {
    float speed = (length - i) / length * 1.2 + i / length * 0.75;
    T = T * glTranslate(speed, 0, 0);
    trajectory_.push_back(T * extrinsicPose_);
  }

  for (uint32_t i = 0; i < 30; ++i) {
    T = T * glRotateZ(Radians(-3.0)) * glTranslate(0.5, 0, 0);
    trajectory_.push_back(T * extrinsicPose_);
  }

  for (uint32_t i = 0; i < 100; ++i) {
    T = T * glTranslate(0.75 + sigma_speed_ * rng_.getGaussianFloat(), 0, 0);
    trajectory_.push_back(T * extrinsicPose_);
  }

  for (uint32_t i = 0; i < 30; ++i) {
    T = T * glRotateZ(Radians(-3.0)) * glTranslate(0.5, 0, 0);
    trajectory_.push_back(T * extrinsicPose_);
  }

  for (uint32_t i = 0; i < 30; ++i) {
    T = T * glTranslate(0.75 + sigma_speed_ * rng_.getGaussianFloat(), 0, 0);
    trajectory_.push_back(T * extrinsicPose_);
  }
}

void SimulationReader::setParameters(const ParameterList& params) {
  sigma_noise_ = params["sigma_noise"];
  sigma_speed_ = params["sigma_speed"];
}

void SimulationReader::reset() {
  timestamp_ = 0;
}

bool SimulationReader::read(rv::Laserscan& scan) {
  scan.clear();
  if (timestamp_ >= trajectory_.size()) return false;
  if (buffer_.size() > timestamp_) {
    scan = buffer_[timestamp_];
    return true;
  }

  Eigen::Matrix4f T = trajectory_[timestamp_];
  Ray ray(T.col(3), Eigen::Vector4f(0, 0, 0, 0));
  // generate synthetic scan for current pose. ( highly inefficient.)
  // idea: render indexmap with appropriate projection matrix; do intersection test only for indexes & store triangles
  // in texture for parallel intersection tests...
  for (uint32_t i = 0; i < beams_.size(); ++i) {
    uint32_t beam_id = beams_[i].w();
    float nearest_range = 100.0f;
    Eigen::Vector4f beam = beams_[i];
    beam[3] = 0.0f;
    ray.d = T * beam;  // get appropriate direction for given pose.

    bool found = false;

    // this part is highly inefficient:
    for (uint32_t j = 0; j < world_.size(); ++j) {
      float t;
      if (intersect(ray, world_[j], t)) {
        nearest_range = std::min(nearest_range, t);
        found = true;
      }
    }

    // extrinsicPose_;
    if (found && nearest_range > 0 && nearest_range < 75.0f) {
      Eigen::Vector4f p =
          T.inverse() *
          ray(nearest_range * calibration_[beam_id]);  // local coordinate system of the laser range scanner.
      scan.points().push_back(Point3f(p[0], p[1], p[2]));
    }
  }

  return true;
}

bool SimulationReader::isSeekable() const {
  return true;
}

void SimulationReader::seek(uint32_t scan) {
  timestamp_ = scan;
}

uint32_t SimulationReader::count() const {
  return trajectory_.size();
}

// https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
bool SimulationReader::intersect(const Ray& ray, const Triangle& triangle, float& t) {
  const float EPSILON = 0.0000000001f;

  // Find vectors for two edges sharing V1
  Eigen::Vector4f e1 = triangle.vertices[1] - triangle.vertices[0];
  Eigen::Vector4f e2 = triangle.vertices[2] - triangle.vertices[0];

  // Begin calculating determinant - also used to calculate u parameter

  Eigen::Vector4f P = ray.d.cross3(e2);
  P[3] = 0;
  // if determinant is near zero, ray lies in plane of triangle or ray is parallel to plane of triangle
  float det = e1.dot(P);
  // NOT CULLING
  if (det > -EPSILON && det < EPSILON) return false;
  float inv_det = 1.f / det;

  // calculate distance from V1 to ray origin
  Eigen::Vector4f T = ray.o - triangle.vertices[0];

  // Calculate u parameter and test bound
  float u = T.dot(P) * inv_det;
  // The intersection lies outside of the triangle
  if (u < 0.f || u > 1.f) return false;

  // Prepare to test v parameter
  Eigen::Vector4f Q = T.cross3(e1);
  Q[3] = 0;

  // Calculate V parameter and test bound
  float v = ray.d.dot(Q) * inv_det;
  // The intersection lies outside of the triangle
  if (v < 0.f || u + v > 1.f) return false;

  t = e2.dot(Q) * inv_det + (sigma_noise_ * rng_.getGaussianFloat());

  return (t > EPSILON);
}

const std::vector<Eigen::Matrix4f>& SimulationReader::getTrajectory() const {
  return trajectory_;
}
