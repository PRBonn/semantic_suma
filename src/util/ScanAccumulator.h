#ifndef INCLUDE_CORE_SCANACCUMULATOR_H_
#define INCLUDE_CORE_SCANACCUMULATOR_H_

#include <glow/GlBuffer.h>
#include <rv/geometry.h>
#include <stdint.h>
#include <vector>

/** \brief Accumulate scans with poses like a ring buffer.
 *
 *  Like in a ring buffer scans are added until a maximum capacity is reached. Then
 *  always the oldest scan is replaced by a new scan.
 *
 *  All scans are stored inside a large GlBuffer. The method getVBO returns the handle to
 *  the vertex buffer object (VBO). The methods offset, pose and size return the offset, pose
 *  and size of the scan with given index, where the last inserted scan has index 0, the scan before
 *  1, etc.
 *
 *  \author behley
 **/
class ScanAccumulator {
 public:
  /** \brief Generate scan accumulator for given numer of scans with maximum number of points,
   *
   * \param history_size  capacity of the scan accumulator, i.e., how many scans are stored before an old scan is
   *replaced by a newer scan.
   * \param max_points    maximum number of points inside a scan.
   */
  ScanAccumulator(uint32_t history_size, uint32_t max_points);

  /** \brief reset accumulator. **/
  void clear();

  /** \brief current number of scans inside the buffer. **/
  uint32_t size() const;

  /** \brief capacity of the accumulator. **/
  uint32_t capacity() const;

  /** \brief offset for scan with given index inside the vertex buffer object. **/
  uint32_t offset(uint32_t idx) const;

  /** \brief pose of the scan with given index. **/
  const Eigen::Matrix4f& pose(uint32_t idx) const;

  /** \brief timestamp of the scan with given index. **/
  uint32_t timestamp(uint32_t idx) const;

  /** \brief number of points for scan with index. **/
  uint32_t size(uint32_t idx) const;

  /** \brief get the associated vertex buffer object. **/
  glow::GlBuffer<rv::Point3f>& getVBO();

  /** \brief inserting given scan and pose. **/
  void insert(uint32_t timestamp, const Eigen::Matrix4f& pose, const std::vector<rv::Point3f>& points);

  /** \brief get poses currently in the ring buffer.**/
  std::vector<Eigen::Matrix4f>& getPoses();
  const std::vector<Eigen::Matrix4f>& getPoses() const;

  /** \brief get timestamps currently in the ring buffer. **/
  std::vector<int32_t>& getTimestamps();
  const std::vector<int32_t>& getTimestamps() const;

 protected:
  glow::GlBuffer<rv::Point3f> vbo_;

  std::vector<Eigen::Matrix4f> poses_;
  std::vector<uint32_t> offsets_;
  std::vector<uint32_t> sizes_;

  std::vector<int32_t> timestamps_;

  uint32_t capacity_;
  uint32_t max_size_;
  uint32_t size_{0};
  int32_t currentIdx_{-1};
};

#endif /* INCLUDE_CORE_SCANACCUMULATOR_H_ */
