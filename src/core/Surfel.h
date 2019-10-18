#ifndef INCLUDE_CORE_SURFEL_H_
#define INCLUDE_CORE_SURFEL_H_

/** \brief surfel representation. **/
struct Surfel {
 public:
  float x, y, z;
  float radius;
  float nx, ny, nz;
  float confidence;

  uint32_t timestamp;
  float color, weight, count;
  float r, g, b, w;
};

#endif /* INCLUDE_CORE_SURFEL_H_ */
