#include "rv/Random.h"

namespace rv
{

Random::Random() :
    uniform_(0.0, 1.0), normal_(0.0, 1.0)
{

}

Random::Random(uint32_t seed) :
    rng_(seed), uniform_(0.0, 1.0), normal_(0.0, 1.0)
{

}

int32_t Random::getInt(int32_t n)
{
  return static_cast<int32_t>(n * uniform_(rng_));
}

int32_t Random::getInt(int32_t min, int32_t max)
{
  return static_cast<int32_t>((max - min) * uniform_(rng_) + min);
}

int32_t Random::operator()(int32_t n)
{
  return getInt(n);
}

float Random::getFloat()
{
  return static_cast<float>(uniform_(rng_));
}

float Random::getGaussianFloat()
{
  return static_cast<float>(uniform_(rng_));
}

double Random::getDouble()
{
  return uniform_(rng_);
}

/** \brief Returns a random double from the Gaussian distribution with mean 0 and std. deviation 1 */
double Random::getGaussianDouble()
{
  return normal_(rng_);
}

}
