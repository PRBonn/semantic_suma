#ifndef LASERSCANREADER_H_
#define LASERSCANREADER_H_

#include <vector>

#include <rv/Laserscan.h>
#include <rv/IOError.h>

namespace rv {

/** \brief laserscan reader interface
 *
 * Defines the interface for a (seekable) laserscan reader.
 *
 * A seekable laser scan should reimplement the seek method.
 *
 * \author behley
 **/
class LaserscanReader {
 public:
  virtual ~LaserscanReader() {}

  /** \brief reset laser scan reader. **/
  virtual void reset() = 0;

  /** \brief read next laser range scan **/
  virtual bool read(Laserscan& scan) = 0;

  /** \brief is reader seekable? **/
  virtual bool isSeekable() const { return false; }

  /** \brief seek to a specific scan number. **/
  virtual void seek(uint32_t scan) { throw IOError("Laserscan reader not seekable."); }

  /** \brief number of scans in the logfile **/
  virtual uint32_t count() const = 0;
};
}
#endif /* LASERSCANREADER_H_ */
