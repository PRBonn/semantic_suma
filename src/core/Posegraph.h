#ifndef SRC_CORE_POSEGRAPH_H_
#define SRC_CORE_POSEGRAPH_H_

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

/**
 *  \author behley
 **/
class Posegraph {
 public:
  typedef std::shared_ptr<Posegraph> Ptr;
  typedef std::shared_ptr<const Posegraph> ConstPtr;
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;
  typedef Eigen::Matrix<float, 6, 6> Matrix6f;

  class Edge {
   public:
    Edge(int32_t from_, int32_t to_, const Eigen::Matrix4d& measurement_)
        : from(from_), to(to_), measurement(measurement_) {}
    int32_t from, to;
    Eigen::Matrix4d measurement;
  };

  Posegraph();

  Posegraph::Ptr clone() const;

  double error() const;

  void clear();

  /** \brief add estimate for a given node \id. **/
  void setInitial(int32_t id, const Eigen::Matrix4d& initial_estimate);

  /** \brief insert edge from node \from to node \to with given measurement (SE3 transform) and information matrix.
   *
   *  The information matrix is the inverse of the covariance, where the
   **/
  void addEdge(int32_t from, int32_t to, const Eigen::Matrix4d& measurement, const Matrix6d& information);

  /** \brief get pose for given id. **/
  Eigen::Matrix4d pose(int32_t id) const;

  /** \brief get (optimized) poses sorted by id of the vertexes. **/
  std::vector<Eigen::Matrix4d> poses() const;

  /** \brief size of graph, i.e., number of vertices **/
  int32_t size() const;

  /** \brief reinitialize model with initial estimates. **/
  void reinitialize();

  /** \brief start optimization for given number of iterations. **/
  bool optimize(uint32_t num_iters);

  void save(const std::string& filename) const;
  void load(const std::string& filename);

  /** \brief set m estimator. **/
  void setMEstimator(const gtsam::noiseModel::mEstimator::Base::shared_ptr& m_estimator);

  const gtsam::NonlinearFactorGraph& graph() const;
  const gtsam::Values& initial() const;
  const gtsam::Values& result() const;

  const std::vector<Edge>& getEdges() const { return edges_; }

 protected:
  std::vector<Edge> edges_;
  gtsam::Values initial_;
  gtsam::Values result_;
  gtsam::NonlinearFactorGraph graph_;

  bool robustify_{false};
  gtsam::noiseModel::mEstimator::Base::shared_ptr robustifier_;
};

#endif /* SRC_CORE_POSEGRAPH_H_ */
