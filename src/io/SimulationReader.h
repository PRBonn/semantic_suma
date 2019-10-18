#ifndef INCLUDE_CORE_SIMULATIONREADER_H_
#define INCLUDE_CORE_SIMULATIONREADER_H_

#include <rv/LaserscanReader.h>
#include <rv/Random.h>
#include <rv/ParameterList.h>

/** \brief simple generator of simulated laser scan data.
 *
 *  \author behley
 **/
class SimulationReader : public rv::LaserscanReader {
 public:
  SimulationReader(const std::string& filename);

  void reset();

  bool read(rv::Laserscan& scan);

  bool isSeekable() const;

  void seek(uint32_t scan);

  uint32_t count() const;

  const std::vector<Eigen::Matrix4f>& getTrajectory() const;

  void setParameters(const rv::ParameterList& params);

 protected:
  class Ray {
   public:
    Ray(const Eigen::Vector4f& origin, const Eigen::Vector4f& dir);
    Eigen::Vector4f operator()(float t) const;

    Eigen::Vector4f o;
    Eigen::Vector4f d;
  };

  class Triangle {
   public:
    Triangle(const Eigen::Vector4f& p1, const Eigen::Vector4f& p2, const Eigen::Vector4f& p3);

    Eigen::Vector4f vertices[3];  // needed to determine if inside plane.
  };

  /** \brief intersect ray with triangle. if true, than t is the parameter needed to calculate the intersection from
   * ray. **/
  bool intersect(const Ray& ray, const Triangle& triangle, float& t);

  void addCube(const Eigen::Matrix4f& pose, float size);
  void addPlane(const Eigen::Matrix4f& pose, float size);

  uint32_t timestamp_{0};
  std::vector<Eigen::Matrix4f> trajectory_;  // sensor locations at time t.
  std::vector<Triangle> world_;              // world is described by triangles.
  std::vector<rv::Laserscan> buffer_;

  std::vector<Eigen::Vector4f> beams_;
  std::vector<float> calibration_;
  Eigen::Matrix4f extrinsicPose_;

  rv::Random rng_;
  float sigma_noise_{0.0f};
  float sigma_speed_{0.0f};
};

#endif /* INCLUDE_CORE_SIMULATIONREADER_H_ */
