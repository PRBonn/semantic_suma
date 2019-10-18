#ifndef KITTILASERSCANREADER_H_
#define KITTILASERSCANREADER_H_

#include "LaserscanReader.h"

#include <rv/ParameterList.h>
#include <rv/RingBuffer.h>
#include <rv/IOError.h>

#include <map>

#include <io/RangenetAPI.hpp>

typedef std::tuple< u_char, u_char, u_char> semantic_color;

namespace rv {

/** \brief a reader for the KITTI datasets provided by the KIT.
 *
 *  \author behley
 *
 *  \author Xieyuanli Chen
 */
class KITTIReader : public LaserscanReader {
 public:
  KITTIReader(const std::string& scan_filename, ParameterList params, uint32_t buffer_size = 50);

  void reset() override;
  bool read(Laserscan& scan) override;
  void seek(uint32_t scan) override;
  bool isSeekable() const override;
  uint32_t count() const override;

  std::map<uint32_t, semantic_color> getColorMap(){ return color_map_; }

 protected:
  std::vector<int> label_map_;
  std::map<uint32_t, semantic_color> color_map_;
  void initScanFilenames(const std::string& scan_filename);

  bool read(uint32_t scan_idx, Laserscan& scan);

  int32_t currentScan;
  std::vector<std::string> scan_filenames;
  RingBuffer<Laserscan> bufferedScans;
  uint32_t firstBufferedScan;

  ParameterList params_;
  std::shared_ptr<RangenetAPI> net;

};
}
#endif /* KITTILASERSCANREADER_H_ */
