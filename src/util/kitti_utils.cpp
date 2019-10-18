#include "kitti_utils.h"

#include <fstream>
#include <vector>
#include <rv/string_utils.h>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

using namespace rv;

#define SAFE_COMMAND(cmd)                        \
  if (!system(cmd)) {                            \
    std::cerr << cmd << " failed." << std::endl; \
    exit(1);                                     \
  }

KITTICalibration::KITTICalibration() {
}

KITTICalibration::KITTICalibration(const std::string& filename) {
  initialize(filename);
}

const Eigen::Matrix4f& KITTICalibration::operator[](const std::string& name) const {
  if (m_.find(name) == m_.end()) throw std::runtime_error("Unknown name for calibration matrix.");

  return m_.find(name)->second;
}

void KITTICalibration::initialize(const std::string& filename) {
  m_.clear();
  std::ifstream cin(filename.c_str());
  if (!cin.is_open()) {
    throw std::runtime_error(std::string("Unable to open calibration file: ") + filename);
  }
  std::string line;

  cin.peek();
  while (!cin.eof()) {
    std::getline(cin, line);
    std::vector<std::string> tokens = rv::split(line, ":");

    if (tokens.size() == 2) {
      std::string name = rv::trim(tokens[0]);
      std::vector<std::string> entries = rv::split(rv::trim(tokens[1]), " ");
      if (entries.size() == 12) {
        Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
        for (uint32_t i = 0; i < 12; ++i) {
          m(i / 4, i - int(i / 4) * 4) = boost::lexical_cast<float>(rv::trim(entries[i]));
        }

        m_[name] = m;
      }
    }
    cin.peek();
  }

  cin.close();
}

void KITTICalibration::clear() {
  m_.clear();
}

bool KITTICalibration::exists(const std::string& name) const {
  return (m_.find(name) != m_.end());
}

namespace KITTI {

namespace Odometry {

float lengths[] = {100, 200, 300, 400, 500, 600, 700, 800};
int32_t num_lengths = 8;

std::vector<Eigen::Matrix4f> loadPoses(const std::string& file_name) {
  std::vector<Eigen::Matrix4f> poses;
  std::ifstream fp(file_name.c_str());
  std::string line;

  if (!fp.is_open()) return poses;
  fp.peek();

  while (fp.good()) {
    Eigen::Matrix4f P = Eigen::Matrix4f::Identity();

    std::getline(fp, line);
    std::vector<std::string> entries = rv::split(line, " ");

    if (entries.size() < 12) {
      fp.peek();
      continue;
    }

    for (uint32_t i = 0; i < 12; ++i) {
      P(i / 4, i - int(i / 4) * 4) = boost::lexical_cast<float>(rv::trim(entries[i]));
    }

    poses.push_back(P);

    fp.peek();
  }

  fp.close();

  return poses;
}

std::vector<float> trajectoryDistances(const std::vector<Eigen::Matrix4f>& poses) {
  std::vector<float> dist;
  dist.push_back(0);
  for (uint32_t i = 1; i < poses.size(); i++) {
    const Eigen::Matrix4f& P1 = poses[i - 1];
    const Eigen::Matrix4f& P2 = poses[i];

    float dx = P1(0, 3) - P2(0, 3);
    float dy = P1(1, 3) - P2(1, 3);
    float dz = P1(2, 3) - P2(2, 3);

    dist.push_back(dist[i - 1] + sqrt(dx * dx + dy * dy + dz * dz));
  }

  return dist;
}

int32_t lastFrameFromSegmentLength(const std::vector<float>& dist, int32_t first_frame, float len) {
  for (uint32_t i = first_frame; i < dist.size(); i++)
    if (dist[i] > dist[first_frame] + len) return i;
  return -1;
}

float rotationError(const Eigen::Matrix4f& pose_error) {
  float a = pose_error(0, 0);
  float b = pose_error(1, 1);
  float c = pose_error(2, 2);
  float d = 0.5 * (a + b + c - 1.0);
  return std::acos(std::max(std::min(d, 1.0f), -1.0f));
}

float translationError(const Eigen::Matrix4f& pose_error) {
  float dx = pose_error(0, 3);
  float dy = pose_error(1, 3);
  float dz = pose_error(2, 3);
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::vector<errors> calcSequenceErrors(const std::vector<Eigen::Matrix4f>& poses_gt,
                                       const std::vector<Eigen::Matrix4f>& poses_result) {
  // error vector
  std::vector<errors> err;

  // parameters
  int32_t step_size = 10;  // every second

  // pre-compute distances (from ground truth as reference)
  std::vector<float> dist = trajectoryDistances(poses_gt);

  // for all start positions do
  for (uint32_t first_frame = 0; first_frame < poses_gt.size(); first_frame += step_size) {
    // for all segment lengths do
    for (int32_t i = 0; i < num_lengths; i++) {
      // current length
      float len = lengths[i];

      // compute last frame
      int32_t last_frame = lastFrameFromSegmentLength(dist, first_frame, len);

      // continue, if sequence not long enough
      if (last_frame == -1) continue;

      // compute rotational and translational errors
      Eigen::Matrix4f pose_delta_gt = poses_gt[first_frame].inverse() * poses_gt[last_frame];
      Eigen::Matrix4f pose_delta_result = poses_result[first_frame].inverse() * poses_result[last_frame];
      Eigen::Matrix4f pose_error = pose_delta_result.inverse() * pose_delta_gt;
      float r_err = rotationError(pose_error);
      float t_err = translationError(pose_error);

      // compute speed
      float num_frames = (float)(last_frame - first_frame + 1);
      float speed = len / (0.1 * num_frames);

      // write to file
      err.push_back(errors(first_frame, r_err / len, t_err / len, len, speed));
    }
  }

  // return error vector
  return err;
}

void saveSequenceErrors(const std::vector<errors>& err, const std::string& file_name) {
  // open file
  //  FILE *fp;
  //  fp = fopen(file_name.c_str(), "w");
  std::ofstream out(file_name.c_str());

  // write to file
  for (std::vector<errors>::const_iterator it = err.begin(); it != err.end(); it++) {
    // fprintf(fp, "%d %f %f %f %f\n", it->first_frame, it->r_err, it->t_err, it->len, it->speed);
    out << it->first_frame << " " << it->r_err << " " << it->t_err << " " << it->len << " " << it->speed << std::endl;
  }

  // close file
  //  fclose (fp);
  out.close();
}

void savePathPlot(const std::vector<Eigen::Matrix4f>& poses_gt, const std::vector<Eigen::Matrix4f>& poses_result,
                  const std::string& file_name) {
  // parameters
  int32_t step_size = 3;

  // open file
  //  FILE *fp = fopen(file_name.c_str(), "w");
  std::ofstream out(file_name.c_str());

  // save x/z coordinates of all frames to file
  for (uint32_t i = 0; i < poses_gt.size(); i += step_size) {
    out << poses_gt[i](0, 3) << " " << poses_gt[i](2, 3) << " ";
    out << poses_result[i](0, 3) << " " << poses_result[i](2, 3) << std::endl;
  }
  // close file
  //  fclose(fp);
  out.close();
}

std::vector<int32_t> computeRoi(const std::vector<Eigen::Matrix4f>& poses_gt,
                                const std::vector<Eigen::Matrix4f>& poses_result) {
  float x_min = std::numeric_limits<int32_t>::max();
  float x_max = std::numeric_limits<int32_t>::min();
  float z_min = std::numeric_limits<int32_t>::max();
  float z_max = std::numeric_limits<int32_t>::min();

  for (std::vector<Eigen::Matrix4f>::const_iterator it = poses_gt.begin(); it != poses_gt.end(); it++) {
    float x = (*it)(0, 3);
    float z = (*it)(2, 3);
    if (x < x_min) x_min = x;
    if (x > x_max) x_max = x;
    if (z < z_min) z_min = z;
    if (z > z_max) z_max = z;
  }

  for (std::vector<Eigen::Matrix4f>::const_iterator it = poses_result.begin(); it != poses_result.end(); it++) {
    float x = (*it)(0, 3);
    float z = (*it)(2, 3);
    if (x < x_min) x_min = x;
    if (x > x_max) x_max = x;
    if (z < z_min) z_min = z;
    if (z > z_max) z_max = z;
  }

  float dx = 1.1 * (x_max - x_min);
  float dz = 1.1 * (z_max - z_min);
  float mx = 0.5 * (x_max + x_min);
  float mz = 0.5 * (z_max + z_min);
  float r = 0.5 * std::max(dx, dz);

  std::vector<int32_t> roi;
  roi.push_back((int32_t)(mx - r));
  roi.push_back((int32_t)(mx + r));
  roi.push_back((int32_t)(mz - r));
  roi.push_back((int32_t)(mz + r));

  return roi;
}

void plotPathPlot(const std::string& dir, const std::vector<int32_t>& roi, int32_t idx) {
  // gnuplot file name
  char command[1024];
  char file_name[256];
  sprintf(file_name, "%02d.gp", idx);
  std::string full_name = dir + "/" + file_name;

  // create png + eps
  for (int32_t i = 0; i < 2; i++) {
    // open file
    FILE* fp = fopen(full_name.c_str(), "w");

    // save gnuplot instructions
    if (i == 0) {
      fprintf(fp, "set term png size 900,900\n");
      fprintf(fp, "set output \"%02d.png\"\n", idx);
    } else {
      fprintf(fp, "set term postscript eps enhanced color\n");
      fprintf(fp, "set output \"%02d.eps\"\n", idx);
    }

    fprintf(fp, "set size ratio -1\n");
    fprintf(fp, "set xrange [%d:%d]\n", roi[0], roi[1]);
    fprintf(fp, "set yrange [%d:%d]\n", roi[2], roi[3]);
    fprintf(fp, "set xlabel \"x [m]\"\n");
    fprintf(fp, "set ylabel \"z [m]\"\n");
    fprintf(fp, "plot \"%02d.txt\" using 1:2 lc rgb \"#FF0000\" title 'Ground Truth' w lines,", idx);
    fprintf(fp, "\"%02d.txt\" using 3:4 lc rgb \"#0000FF\" title 'Visual Odometry' w lines,", idx);
    fprintf(fp, "\"< head -1 %02d.txt\" using 1:2 lc rgb \"#000000\" pt 4 ps 1 lw 2 title 'Sequence Start' w points\n",
            idx);

    // close file
    fclose(fp);

    // run gnuplot => create png + eps
    sprintf(command, "cd %s; gnuplot %s", dir.c_str(), file_name);
    if (!system(command)) exit(1);
  }

  // create pdf and crop
  sprintf(command, "cd %s; ps2pdf %02d.eps %02d_large.pdf", dir.c_str(), idx, idx);
  SAFE_COMMAND(command);
  sprintf(command, "cd %s; pdfcrop %02d_large.pdf %02d.pdf", dir.c_str(), idx, idx);
  SAFE_COMMAND(command);
  sprintf(command, "cd %s; rm %02d_large.pdf", dir.c_str(), idx);
  SAFE_COMMAND(command);
}

void saveErrorPlots(const std::vector<errors>& seq_err, const std::string& plot_error_dir, const std::string& prefix) {
  // file names
  char file_name_tl[1024];
  sprintf(file_name_tl, "%s/%s_tl.txt", plot_error_dir.c_str(), prefix.c_str());
  char file_name_rl[1024];
  sprintf(file_name_rl, "%s/%s_rl.txt", plot_error_dir.c_str(), prefix.c_str());
  char file_name_ts[1024];
  sprintf(file_name_ts, "%s/%s_ts.txt", plot_error_dir.c_str(), prefix.c_str());
  char file_name_rs[1024];
  sprintf(file_name_rs, "%s/%s_rs.txt", plot_error_dir.c_str(), prefix.c_str());

  // open files
  FILE* fp_tl = fopen(file_name_tl, "w");
  FILE* fp_rl = fopen(file_name_rl, "w");
  FILE* fp_ts = fopen(file_name_ts, "w");
  FILE* fp_rs = fopen(file_name_rs, "w");

  // for each segment length do
  for (int32_t i = 0; i < num_lengths; i++) {
    float t_err = 0;
    float r_err = 0;
    float num = 0;

    // for all errors do
    for (std::vector<errors>::const_iterator it = seq_err.begin(); it != seq_err.end(); it++) {
      if (fabs(it->len - lengths[i]) < 1.0) {
        t_err += it->t_err;
        r_err += it->r_err;
        num++;
      }
    }

    // we require at least 3 values
    if (num > 2.5) {
      fprintf(fp_tl, "%f %f\n", lengths[i], t_err / num);
      fprintf(fp_rl, "%f %f\n", lengths[i], r_err / num);
    }
  }

  // for each driving speed do (in m/s)
  for (float speed = 2; speed < 25; speed += 2) {
    float t_err = 0;
    float r_err = 0;
    float num = 0;

    // for all errors do
    for (std::vector<errors>::const_iterator it = seq_err.begin(); it != seq_err.end(); it++) {
      if (fabs(it->speed - speed) < 2.0) {
        t_err += it->t_err;
        r_err += it->r_err;
        num++;
      }
    }

    // we require at least 3 values
    if (num > 2.5) {
      fprintf(fp_ts, "%f %f\n", speed, t_err / num);
      fprintf(fp_rs, "%f %f\n", speed, r_err / num);
    }
  }

  // close files
  fclose(fp_tl);
  fclose(fp_rl);
  fclose(fp_ts);
  fclose(fp_rs);
}

void plotErrorPlots(const std::string& dir, char* prefix) {
  char command[1024];

  // for all four error plots do
  for (int32_t i = 0; i < 4; i++) {
    // create suffix
    char suffix[16];
    switch (i) {
      case 0:
        sprintf(suffix, "tl");
        break;
      case 1:
        sprintf(suffix, "rl");
        break;
      case 2:
        sprintf(suffix, "ts");
        break;
      case 3:
        sprintf(suffix, "rs");
        break;
    }

    // gnuplot file name
    char file_name[1024];
    char full_name[1024];
    sprintf(file_name, "%s_%s.gp", prefix, suffix);
    sprintf(full_name, "%s/%s", dir.c_str(), file_name);

    // create png + eps
    for (int32_t j = 0; j < 2; j++) {
      // open file
      FILE* fp = fopen(full_name, "w");

      // save gnuplot instructions
      if (j == 0) {
        fprintf(fp, "set term png size 500,250 font \"Helvetica\" 11\n");
        fprintf(fp, "set output \"%s_%s.png\"\n", prefix, suffix);
      } else {
        fprintf(fp, "set term postscript eps enhanced color\n");
        fprintf(fp, "set output \"%s_%s.eps\"\n", prefix, suffix);
      }

      // start plot at 0
      fprintf(fp, "set size ratio 0.5\n");
      fprintf(fp, "set yrange [0:*]\n");

      // x label
      if (i <= 1)
        fprintf(fp, "set xlabel \"Path Length [m]\"\n");
      else
        fprintf(fp, "set xlabel \"Speed [km/h]\"\n");

      // y label
      if (i == 0 || i == 2)
        fprintf(fp, "set ylabel \"Translation Error [%%]\"\n");
      else
        fprintf(fp, "set ylabel \"Rotation Error [deg/m]\"\n");

      // plot error curve
      fprintf(fp, "plot \"%s_%s.txt\" using ", prefix, suffix);
      switch (i) {
        case 0:
          fprintf(fp, "1:($2*100) title 'Translation Error'");
          break;
        case 1:
          fprintf(fp, "1:($2*57.3) title 'Rotation Error'");
          break;
        case 2:
          fprintf(fp, "($1*3.6):($2*100) title 'Translation Error'");
          break;
        case 3:
          fprintf(fp, "($1*3.6):($2*57.3) title 'Rotation Error'");
          break;
      }
      fprintf(fp, " lc rgb \"#0000FF\" pt 4 w linespoints\n");

      // close file
      fclose(fp);

      // run gnuplot => create png + eps
      sprintf(command, "cd %s; gnuplot %s", dir.c_str(), file_name);
      SAFE_COMMAND(command);
    }

    // create pdf and crop
    sprintf(command, "cd %s; ps2pdf %s_%s.eps %s_%s_large.pdf", dir.c_str(), prefix, suffix, prefix, suffix);
    SAFE_COMMAND(command);
    sprintf(command, "cd %s; pdfcrop %s_%s_large.pdf %s_%s.pdf", dir.c_str(), prefix, suffix, prefix, suffix);
    SAFE_COMMAND(command);
    sprintf(command, "cd %s; rm %s_%s_large.pdf", dir.c_str(), prefix, suffix);
    SAFE_COMMAND(command);
  }
}

void saveStats(const std::vector<errors>& err, const std::string& dir) {
  float t_err = 0;
  float r_err = 0;

  // for all errors do => compute sum of t_err, r_err
  for (std::vector<errors>::const_iterator it = err.begin(); it != err.end(); it++) {
    t_err += it->t_err;
    r_err += it->r_err;
  }

  // open file
  FILE* fp = fopen((dir + "/stats.txt").c_str(), "w");

  // save errors
  float num = err.size();
  fprintf(fp, "%f %f\n", t_err / num, r_err / num);

  // close file
  fclose(fp);
}

/** \brief evaluating the odometry results for given result_dir and gt_dir.
 *
 *  \param gt_dir groundtruth directory containing XX.txt files.
 *  \param result_dir directory where the estimated trajectory is stored. This is also the directory
 *      where all files are written.
 **/
bool eval(const std::string& gt_dir, const std::string& result_dir) {
  // ground truth and result directories
  //  string gt_dir = "data/odometry/poses";
  //  string result_dir = "results/" + result_sha;
  std::string error_dir = result_dir + "/errors";
  std::string plot_path_dir = result_dir + "/plot_path";
  std::string plot_error_dir = result_dir + "/plot_error";

  // create output directories
  SAFE_COMMAND(("mkdir " + error_dir).c_str());
  SAFE_COMMAND(("mkdir " + plot_path_dir).c_str());
  SAFE_COMMAND(("mkdir " + plot_error_dir).c_str());

  // total errors
  std::vector<errors> total_err;

  // for all sequences do
  for (int32_t i = 11; i < 22; i++) {
    // file name
    char file_name[256];
    sprintf(file_name, "%02d.txt", i);

    // read ground truth and result poses
    std::vector<Eigen::Matrix4f> poses_gt = loadPoses(gt_dir + "/" + file_name);
    std::vector<Eigen::Matrix4f> poses_result = loadPoses(result_dir + "/data/" + file_name);

    // plot status
    std::cout << "Processing: " << file_name << ", poses: " << poses_result.size() << "/" << poses_gt.size()
              << std::endl;

    // check for errors
    if (poses_gt.size() == 0 || poses_result.size() != poses_gt.size()) {
      std::cout << "ERROR: Couldn't read (all) poses of: " << file_name << std::endl;
      return false;
    }

    // compute sequence errors
    std::vector<errors> seq_err = calcSequenceErrors(poses_gt, poses_result);
    saveSequenceErrors(seq_err, error_dir + "/" + file_name);

    // add to total errors
    total_err.insert(total_err.end(), seq_err.begin(), seq_err.end());

    // for first half => plot trajectory and compute individual stats
    if (i <= 15) {
      // save + plot bird's eye view trajectories
      savePathPlot(poses_gt, poses_result, plot_path_dir + "/" + file_name);
      std::vector<int32_t> roi = computeRoi(poses_gt, poses_result);
      plotPathPlot(plot_path_dir, roi, i);

      // save + plot individual errors
      char prefix[16];
      sprintf(prefix, "%02d", i);
      saveErrorPlots(seq_err, plot_error_dir, prefix);
      plotErrorPlots(plot_error_dir, prefix);
    }
  }

  // save + plot total errors + summary statistics
  if (total_err.size() > 0) {
    char prefix[16];
    sprintf(prefix, "avg");
    saveErrorPlots(total_err, plot_error_dir, prefix);
    plotErrorPlots(plot_error_dir, prefix);
    saveStats(total_err, result_dir);
  }

  // success
  return true;
}
}
}
