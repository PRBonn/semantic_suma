#ifndef SRC_IO_ROBOCARREADER_H_
#define SRC_IO_ROBOCARREADER_H_

#include "LaserscanReader.h"

namespace rv {

/** \brief Laserscan reader for RoboCar dataset.
 *
 *  \author behley
 **/
class RobocarReader : public LaserscanReader {
 public:
  RobocarReader(const std::string& filename);

  void reset();

  bool read(rv::Laserscan& scan);

  bool isSeekable() const;

  void seek(uint32_t scan);

  uint32_t count() const;

 protected:
  uint32_t currentIdx_{0};
  std::vector<std::string> filenames_;
};
}

#endif /* SRC_IO_ROBOCARREADER_H_ */
