#ifndef CORE_POINT_TRAITS_H_
#define CORE_POINT_TRAITS_H_

/** \brief 
 *  
 * Some traits to access coordinates regardless of the specific implementation of point
 * inspired by boost.geometry, which needs to be implemented by new points.
 * 
 *  \author behley
 */

namespace rv
{

namespace traits
{

template<typename PointT, int D>
struct access
{
};

template<class PointT>
struct access<PointT, 0>
{
    static float get(const PointT& p)
    {
      return p.x;
    }

    static void set(PointT& p, float value)
    {
      p.x = value;
    }
};

template<class PointT>
struct access<PointT, 1>
{
    static float get(const PointT& p)
    {
      return p.y;
    }

    static void set(PointT& p, float value)
    {
      p.y = value;
    }
};

template<class PointT>
struct access<PointT, 2>
{
    static float get(const PointT& p)
    {
      return p.z;
    }

    static void set(PointT& p, float value)
    {
      p.z = value;
    }
};

} // namespace traits

/** convenience function for access of point coordinates **/
template<int D, typename PointT>
inline float get(const PointT& p)
{
  return traits::access<PointT, D>::get(p);
}

template<int D, typename PointT>
inline void set(PointT& p, float value)
{
  traits::access<PointT, D>::set(p, value);
}

} // namespace rv

#endif /* SRC_CORE_RV_POINT_TRAITS_H_ */
