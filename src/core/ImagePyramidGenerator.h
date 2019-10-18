#ifndef SRC_CORE_IMAGEPYRAMIDGENERATOR_H_
#define SRC_CORE_IMAGEPYRAMIDGENERATOR_H_

#include <glow/glutil.h>
#include <vector>

/** \brief generate a image pyramid by reordering image coordinates.
 *
 *  To get a simple way of addressing the different layers of a image pyramid,
 *  we want to reorder the given image coordinates, such that the coordinates are
 *  ordered according to the level from coarse-to-fine. This means the array of
 *  image coordinates contains in the beginning all points shared by all levels.
 *  Directly following are the coordinates of the remaining letters.
 *
 *  Output is the reordered array of image coordinates and a sizes array. You can
 *  now access the resulting array like img_coords[0:sizes[level]] and get
 *  the corresponding image coordinate at the specific level from coarse to fine.
 *
 *  This particular order should make it easy to evaluate the residual at different
 *  levels using a vertex shader. Another advantage is that we only need a single
 *  array of image coordinates.
 *
 *  In contrast to the usual downsampling, we use always the exact same image
 *  coordinates and do not apply something like a Gaussian blur.
 *
 *  \author behley
 */
class ImagePyramidGenerator {
 public:
  /** \brief Generate pyramid from given img_coords, returning reordered img_coords and sizes of each level.
   *
   *  if it is not possible to generate max_levels of subsampled images, we simply resize sizes to max level.
   *
   *  \param max_levels specifies how many levels we want to have in the end.
   *  \param width, height  of the initial image.
   *  \param imag_coords array of image coords, which will be reordered.
   *  \param size of resulting images at given level ordered from coarse to fine.
   *
   **/
  static void makePyramid(uint32_t max_levels, uint32_t width, uint32_t height, std::vector<glow::vec2>& img_coords,
                          std::vector<glow::vec2>& sizes);

 protected:
  /** \brief recursively reorder img_coords to generate pyramid of image coordinates.
   *
   *  never called directly.
   *
   * \param level currently generated level.
   * \param img_coords reordered image coordinates.
   * \param sizes array of (width, height) of each level of the pyramid.
   */
  static int32_t shuffle(uint32_t level, std::vector<glow::vec2>& img_coords, std::vector<glow::vec2>& sizes);
};

#endif /* SRC_CORE_IMAGEPYRAMIDGENERATOR_H_ */
