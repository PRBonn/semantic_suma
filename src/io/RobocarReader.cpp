#include "RobocarReader.h"

#include <rv/FileUtil.h>
#include <fstream>

namespace rv
{

RobocarReader::RobocarReader(const std::string& filename) {
  std::vector<std::string> files = FileUtil::getDirectoryListing(FileUtil::dirName(filename));

  for (uint32_t i = 0; i < files.size(); ++i) {
    if (FileUtil::extension(files[i]) == ".bin") {
      filenames_.push_back(files[i]);
    }
  }
  std::cout << "Found " << filenames_.size() << " scans." << std::endl;
  std::sort(filenames_.begin(), filenames_.end());
}

void RobocarReader::reset() {
  currentIdx_ = 0;
}

bool RobocarReader::read(Laserscan& scan) {
  if (currentIdx_ >= filenames_.size()) return false;

  std::ifstream in(filenames_[currentIdx_].c_str(), std::ios::binary);
  if (!in.is_open()) return false;

  scan.clear();

  in.seekg(0, std::ios::end);
  uint32_t num_points = in.tellg() / (3 * sizeof(double));
  in.seekg(0, std::ios::beg);

  std::vector<double> values(3 * num_points);
  in.read((char*)&values[0], 3 * num_points * sizeof(double));

  in.close();
  std::vector<Point3f>& points = scan.points();

  points.resize(num_points);

  for (uint32_t i = 0; i < num_points; ++i) {
    points[i].x() = values[3 * i];
    points[i].y() = -values[3 * i + 1];
    points[i].z() = -values[3 * i + 2];
  }

  currentIdx_ += 1;

  return true;
}

bool RobocarReader::isSeekable() const {
  return true;
}

void RobocarReader::seek(uint32_t scan) {
  currentIdx_ = scan;
}

uint32_t RobocarReader::count() const {
  return filenames_.size();
}

}
