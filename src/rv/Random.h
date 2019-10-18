#ifndef CORE_RANDOM_H_
#define CORE_RANDOM_H_

#include <random>

namespace rv
{

/** \brief random number generator based on boost::random
 *  
 *  The class simply decorates the <random> functionality in a more usable way.
 *
 * 
 *  \author snej
 */

class Random
{
  public:
    Random();
    explicit Random(uint32_t seed);

    /** \brief Returns a random integer value in the range [0,\a n - 1] inclusive. */
    int32_t getInt(int32_t n);
    /** \brief Returns a random integer value in the range [\a min,\a max] inclusive. */
    int32_t getInt(int32_t min, int32_t max);

    /** \brief Function operator for use with STL.
     *
     * Returns a random value between 0 and \a n - 1 inclusive.
     */
    int32_t operator()(int32_t n);

    /** \brief Returns a random float in the range [0,1] inclusive. */
    float getFloat();
    /** \brief Returns a random float from the Gaussian distribution with mean 0 and std. deviation 1 */
    float getGaussianFloat();
    /** \brief Returns a random double in the range [0,1] inclusive. */
    double getDouble();
    /** \brief Returns a random double from the Gaussian distribution with mean 0 and std. deviation 1 */
    double getGaussianDouble();

  protected:
    std::default_random_engine rng_;
    std::uniform_real_distribution<double> uniform_;
    std::normal_distribution<double> normal_;
};

}

#endif /* CORE_RANDOM_H_ */
