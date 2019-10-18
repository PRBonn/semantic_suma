/**
 * \brief representation of a laser range scan with points + remission and optional normals.
 *
 * The remission is assumed to give values in [0.0,1.0]
 *
 * \author behley
 */

#ifndef LASERSCAN_H_
#define LASERSCAN_H_

#include "geometry.h"
#include "transform.h"
#include <stdint.h>
#include <vector>
#include <list>
#include <opencv2/core/core.hpp>

namespace rv
{

class Laserscan
{
  public:
    friend class LaserscanReader;
    friend class KITTIReader;
    friend class PCAPReader;
    friend class EZDReader;
    friend class BLFReader;
    friend class FreiburgReader;

    Laserscan();
    Laserscan(const Transform& pose, const std::list<Point3f>& points,
        const std::list<float>& remission);

    // for semantic map
    Laserscan(const Transform& pose, const std::list<Point3f>& points,
        const std::list<float>& remission, const std::list<Point3f>& labels);

    /** \brief clear **/
    void clear();

    /** \brief getter for points/normals/remission **/
    const Point3f& point(uint32_t i) const;
    float remission(uint32_t i) const;
    const Normal3f& normal(uint32_t) const;

    /** \brief global pose of the sensor. **/
    Transform& pose();
    const Transform& pose() const;
    /** \brief number of points. **/
    uint32_t size() const;

    /** access to the raw data. **/
    std::vector<Point3f>& points();
    const std::vector<Point3f>& points() const;
    std::vector<float>& remissions();
    const std::vector<float>& remissions() const;
    std::vector<Normal3f>& normals();
    const std::vector<Normal3f>& normals() const;

    bool hasRemission() const;
    bool hasNormals() const;

    // for semantic map
    uchar* color_mask = nullptr;    
    std::vector<Point3f> labels_;
    std::vector<float> labels_float;
    std::vector<float> labels_prob;
    
    const Point3f& label(uint32_t i) const;

    std::vector<Point3f>& labels();
    const std::vector<Point3f>& labels() const;

    bool hasLabels() const;

  protected:
    Transform mPose;
    /** representing points, normals, and remission this way, we can separately initialize points and normals **/
    std::vector<Point3f> points_;
    std::vector<float> remissions_;
    std::vector<Normal3f> normals_;
};

}

#endif /* LASERSCAN_H_ */
