#ifndef INCLUDE_CORE_KITTI_UTILS_H_
#define INCLUDE_CORE_KITTI_UTILS_H_

#include <string>
#include <map>
#include <vector>
#include <eigen3/Eigen/Dense>

/** \brief parse calibration file given in KITTI file format
 *  \author behley
 */
class KITTICalibration {
 public:
  KITTICalibration();
  KITTICalibration(const std::string& filename);

  /** \brief initialize calibration matrices from given file in KITTI format. **/
  void initialize(const std::string& filename);

  /** \brief remove all matrices. **/
  void clear();

  /** \brief get calibration matrix with given name. **/
  const Eigen::Matrix4f& operator[](const std::string& name) const;

  /** \brief does the calibration matrix with given name exists? **/
  bool exists(const std::string& name) const;

 protected:
  std::map<std::string, Eigen::Matrix4f> m_;
};

namespace KITTI {

/** \brief  helper functions from devkit for visual odometry evaluation.
 *
 *  Uses Eigen:Matrix4f instead of Matrix.
 */
namespace Odometry {

struct errors {
  int32_t first_frame;
  float r_err;
  float t_err;
  float len;
  float speed;
  errors(int32_t first_frame, float r_err, float t_err, float len, float speed)
      : first_frame(first_frame), r_err(r_err), t_err(t_err), len(len), speed(speed) {}
};

/** \brief load poses from text file. **/
std::vector<Eigen::Matrix4f> loadPoses(const std::string& file_name);

std::vector<float> trajectoryDistances(const std::vector<Eigen::Matrix4f>& poses);

int32_t lastFrameFromSegmentLength(const std::vector<float>& dist, int32_t first_frame, float len);

float rotationError(const Eigen::Matrix4f& pose_error);

float translationError(const Eigen::Matrix4f& pose_error);

std::vector<errors> calcSequenceErrors(const std::vector<Eigen::Matrix4f>& poses_gt,
                                       const std::vector<Eigen::Matrix4f>& poses_result);

void saveSequenceErrors(const std::vector<errors>& err, const std::string& file_name);

void savePathPlot(const std::vector<Eigen::Matrix4f>& poses_gt, const std::vector<Eigen::Matrix4f>& poses_result,
                  const std::string& file_name);

std::vector<int32_t> computeRoi(const std::vector<Eigen::Matrix4f>& poses_gt,
                                const std::vector<Eigen::Matrix4f>& poses_result);

void plotPathPlot(const std::string& dir, const std::vector<int32_t>& roi, int32_t idx);

void saveErrorPlots(const std::vector<errors>& seq_err, const std::string& plot_error_dir, const std::string& prefix);

void plotErrorPlots(const std::string& dir, char* prefix);

void saveStats(const std::vector<errors>& err, const std::string& dir);

/** \brief evaluating the odometry results for given result_dir and gt_dir.
 *
 *  \param gt_dir groundtruth directory containing XX.txt files.
 *  \param result_dir directory where the estimated trajectory is stored. This is also the directory
 *      where all files are written.
 **/
bool eval(const std::string& gt_dir, const std::string& result_dir);
}
}

#endif /* INCLUDE_CORE_KITTI_UTILS_H_ */
