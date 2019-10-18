#include "KITTIReader.h"

#include <rv/FileUtil.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <thread>

#include <glow/glutil.h>
#include <rv/XmlDocument.h>
#include <rv/string_utils.h>
#include <boost/lexical_cast.hpp>

#include <rv/PrimitiveParameters.h>

namespace rv {

KITTIReader::KITTIReader(const std::string& scan_filename, ParameterList params, uint32_t buffer_size)
    : currentScan(0), bufferedScans(buffer_size), firstBufferedScan(0) {

  params_ = params;

  initScanFilenames(scan_filename);

  // initialize the rangenet
  net = std::shared_ptr<RangenetAPI> (new RangenetAPI(params_));
  label_map_ = net->getLabelMap();
  color_map_ = net->getColorMap();
}

void KITTIReader::initScanFilenames(const std::string& scan_filename) {
  scan_filenames.clear();

  std::vector<std::string> files = FileUtil::getDirectoryListing(FileUtil::dirName(scan_filename));
  /** filter irrelevant files. **/
  for (uint32_t i = 0; i < files.size(); ++i) {
    if (FileUtil::extension(files[i]) == ".bin") {
      scan_filenames.push_back(files[i]);
    }
  }

  std::sort(scan_filenames.begin(), scan_filenames.end());
}

void KITTIReader::reset() {
  currentScan = 0;
  bufferedScans.clear();
  firstBufferedScan = 0;
}

bool KITTIReader::read(Laserscan& scan) {
  bool result = false;
  scan.clear();

  if (currentScan >= (int32_t)scan_filenames.size()) {
    return false;
  }

  if (currentScan - firstBufferedScan < bufferedScans.size()) /** scan already in buffer, no need to read scan. **/
  {
    scan = bufferedScans[currentScan - firstBufferedScan];

    result = true;
  } else {
    result = read(currentScan, scan);
    if (result) {
      if (bufferedScans.capacity() == bufferedScans.size()) ++firstBufferedScan;
      bufferedScans.push_back(scan);
    }
  }

  ++currentScan;

  return result;
}

bool KITTIReader::isSeekable() const {
  return true;
}

void KITTIReader::seek(uint32_t scannr) {
  assert(scannr < scan_filenames.size());

  /** scan already in buffer, nothing to read just set current scan **/
  if (scannr - firstBufferedScan < bufferedScans.size()) {
    currentScan = scannr;
  } else if (currentScan < (int32_t)scannr) {
    /** if we don't have to read everything again than read missing scans. **/
    if ((scannr - 1) - currentScan < bufferedScans.capacity()) {
      currentScan =
          firstBufferedScan + bufferedScans.size(); /** advance to last scan in buffer to read no scans twice. **/
      while (currentScan < (int32_t)scannr) {
        Laserscan scan;

        if (bufferedScans.capacity() == bufferedScans.size()) ++firstBufferedScan;
        read(currentScan, scan);
        bufferedScans.push_back(scan);

        ++currentScan;
      }
    } else /** otherwise we just reset the buffer and start buffering again. **/
    {
      currentScan = scannr;
      firstBufferedScan = scannr;
      bufferedScans.clear();
    }
  } else if (currentScan > (int32_t)scannr) /** we have to add scans at the beginning **/
  {
    /** if we don't have to read every thing new, than read missing scans. **/
    if (currentScan - scannr < bufferedScans.capacity()) {
      currentScan = firstBufferedScan;

      while (currentScan > (int32_t)scannr) {
        --currentScan;

        Laserscan scan;

        read(currentScan, scan);
        bufferedScans.push_front(scan);
      }

      firstBufferedScan = currentScan;
    } else /** otherwise we just reset the buffer and start buffering again. **/
    {
      currentScan = scannr;
      firstBufferedScan = scannr;
      bufferedScans.clear();
    }
  }
}

uint32_t KITTIReader::count() const {
  return scan_filenames.size();
}

bool KITTIReader::read(uint32_t scan_idx, Laserscan& scan) {

  if (scan_idx > scan_filenames.size()) return false;

  std::ifstream in(scan_filenames[scan_idx].c_str(), std::ios::binary);
  if (!in.is_open()) return false;

  scan.clear();

  in.seekg(0, std::ios::end);
  uint32_t num_points = in.tellg() / (4 * sizeof(float));
  in.seekg(0, std::ios::beg);

  std::vector<float> values(4 * num_points);
  in.read((char*)&values[0], 4 * num_points * sizeof(float));

  in.close();
  std::vector<Point3f>& points = scan.points_;
  std::vector<float>& remissions = scan.remissions_;

  points.resize(num_points);
  remissions.resize(num_points);

  float max_remission = 0;

  for (uint32_t i = 0; i < num_points; ++i) {
    points[i].x() = values[4 * i];
    points[i].y() = values[4 * i + 1];
    points[i].z() = values[4 * i + 2];
    remissions[i] = values[4 * i + 3];
    max_remission = std::max(remissions[i], max_remission);
  }

  for (uint32_t i = 0; i < num_points; ++i) {
    remissions[i] /= max_remission;
  }
  // for semantic map
  std::vector<float> color_mask;

  std::vector<std::vector<float>> semantic_points = net->infer(values, num_points);

  for (auto point : semantic_points) {
    for (float ele : point){
      color_mask.push_back(ele);
    }
  }

  std::vector<float>& labels = scan.labels_float;
  std::vector<float>& labels_prob = scan.labels_prob;

  labels.resize(num_points);
  labels_prob.resize(num_points);

  for (uint32_t i = 0; i < num_points; ++i) {
    labels[i] = 0;
    labels_prob[i] = 0;
    for (uint32_t j = 0; j < 20; ++j)
    {
      if (labels_prob[i] <= color_mask[i*20+j])
      {
        labels[i] = label_map_[j];
        labels_prob[i] = color_mask[i*20+j];
      }
    }
  }

  return true;
}
}
