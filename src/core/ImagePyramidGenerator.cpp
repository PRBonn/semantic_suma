#include <core/ImagePyramidGenerator.h>

void ImagePyramidGenerator::makePyramid(uint32_t max_levels, uint32_t width, uint32_t height,
                                        std::vector<glow::vec2>& img_coords, std::vector<glow::vec2>& sizes) {
  sizes.resize(max_levels);
  std::fill(sizes.begin(), sizes.end(), glow::vec2(-1, -1));

  sizes[0] = glow::vec2(width, height);

  uint32_t new_max_level = shuffle(1, img_coords, sizes);

  // reorder and shrink.
  std::reverse(&sizes[0], &sizes[new_max_level]);
  sizes.resize(new_max_level);
}

int32_t ImagePyramidGenerator::shuffle(uint32_t level, std::vector<glow::vec2>& img_coords,
                                       std::vector<glow::vec2>& sizes) {
  if (level >= sizes.size()) return level;  // maximum level reached.

  uint32_t w_old = sizes[level - 1].x, h_old = sizes[level - 1].y;
  if (w_old == 1 || h_old == 1) return level;  // not sensible to devide further.

  std::vector<glow::vec2> temp(w_old * h_old);

  uint32_t w_new = w_old / 2, h_new = h_old;

  uint32_t stride = 2 << (level - 1);

  uint32_t next_level = 0;
  uint32_t prev_level = w_old * h_old - 1;
  for (uint32_t i = 0; i < w_old * h_old; ++i) {
    uint32_t x = img_coords[i].x;
//    uint32_t y = img_coords[i].y;
    if (x % stride == 0) {
      temp[next_level++] = img_coords[i];
    } else {
      temp[prev_level--] = img_coords[i];
    }
  }

  sizes[level] = glow::vec2(w_new, h_new);
  for (uint32_t i = 0; i < temp.size(); ++i) {
    img_coords[i] = temp[i];
  }

  return shuffle(level + 1, img_coords, sizes);
}
