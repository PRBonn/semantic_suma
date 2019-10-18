/*
 pbrt source code Copyright(c) 1998-2010 Matt Pharr and Greg Humphreys.

 This file is part of pbrt.

 pbrt is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.  Note that the text contents of
 the book "Physically Based Rendering" are *not* licensed under the
 GNU GPL.

 pbrt is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 */

/**
 * Distinguish between points, vectors, and normals. Especially, normals need
 * non-standard implementation of transformations.
 *
 * For further information:
 *    Matt Pharr, Greg Humphreys.
 *    Physically Based Rendering - From Theory to Implementation.
 *    Morgan Kaufmann, 2010.
 */

#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <cassert>
#include <cmath>
#include <stdint.h>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "point_traits.h"

namespace rv
{

// Global Inline Functions
inline float Lerp(float t, float v1, float v2)
{
  return (1.f - t) * v1 + t * v2;
}

inline float Clamp(float val, float low, float high)
{
  if (val < low)
    return low;
  else if (val > high)
    return high;
  else
    return val;
}

inline int32_t Clamp(int32_t val, int32_t low, int32_t high)
{
  if (val < low)
    return low;
  else if (val > high)
    return high;
  else
    return val;
}

inline int Mod(int32_t a, int32_t b)
{
  int32_t n = int32_t(a / b);
  a -= n * b;
  if (a < 0) a += b;
  return a;
}

inline float Radians(float deg)
{
  return ((float) M_PI / 180.f) * deg;
}

inline float Degrees(float rad)
{
  return (180.f / (float) M_PI) * rad;
}

inline float Log2(float x)
{
  static float invLog2 = 1.f / logf(2.f);
  return logf(x) * invLog2;
}

inline int Floor2Int(float val);
inline int Log2Int(float v)
{
  return Floor2Int(Log2(v));
}

inline bool IsPowerOf2(int32_t v)
{
  return (v & (v - 1)) == 0;
}

inline uint32_t RoundUpPow2(uint32_t v)
{
  v--;
  v |= v >> 1;
  v |= v >> 2;
  v |= v >> 4;
  v |= v >> 8;
  v |= v >> 16;
  return v + 1;
}

inline int Floor2Int(float val)
{
  return (int) floorf(val);
}

inline int Round2Int(float val)
{
  return Floor2Int(val + 0.5f);
}

inline int Float2Int(float val)
{
  return (int) val;
}

inline int Ceil2Int(float val)
{
  return (int) ceilf(val);
}

struct Vector3f;
struct Point3f;
struct Normal3f;

/** \brief representation of a vector and arithmetic operations **/
struct Vector3f
{
  public:
    Vector3f()
        : vec(0.0f, 0.0f, 0.0f, 0.0f) //, x(vec[0]), y(vec[1]), z(vec[2])
    {

    }

    Vector3f(float xx, float yy, float zz)
        : vec(xx, yy, zz, 0.0f) //, x(vec[0]), y(vec[1]), z(vec[2])
    {
      assert(!HasNaNs());
    }

    bool HasNaNs() const
    {
      return std::isnan(vec[0]) || std::isnan(vec[1]) || std::isnan(vec[2]);
    }

    explicit Vector3f(const Point3f &p);

    Vector3f(const Vector3f &v)
        : vec(v.vec[0], v.vec[1], v.vec[2], 0.0f) //, x(vec[0]), y(vec[1]), z(vec[2])
    {
      assert(!v.HasNaNs());
    }

    Vector3f &operator=(const Vector3f &v)
    {
      assert(!v.HasNaNs());
      vec = v.vec;

      return *this;
    }

    Vector3f operator+(const Vector3f &v) const
    {
      assert(!v.HasNaNs());
      Eigen::Vector4f res = vec + v.vec;

      return Vector3f(res[0], res[1], res[2]);
    }

    Vector3f& operator+=(const Vector3f &v)
    {
      assert(!v.HasNaNs());

      vec += v.vec;

      return *this;
    }

    Vector3f operator-(const Vector3f &v) const
    {
      assert(!v.HasNaNs());

      Eigen::Vector4f res = vec - v.vec;

      return Vector3f(res[0], res[1], res[2]);
    }

    Vector3f& operator-=(const Vector3f &v)
    {
      assert(!v.HasNaNs());

      vec -= v.vec;

      return *this;
    }
    Vector3f operator*(float f) const
    {
      Eigen::Vector4f res = f * vec;

      return Vector3f(res[0], res[1], res[2]);
    }

    Vector3f &operator*=(float f)
    {
      assert(!std::isnan(f));

      vec *= f;

      return *this;
    }

    Vector3f operator/(float f) const
    {
      assert(f != 0);
      float inv = 1.f / f;
      Eigen::Vector4f res = inv * vec;

      return Vector3f(res[0], res[1], res[2]);
    }

    Vector3f &operator/=(float f)
    {
      assert(f != 0);

      float inv = 1.f / f;
      vec *= inv;

      return *this;
    }

    Vector3f operator-() const
    {
      return Vector3f(-vec[0], -vec[1], -vec[2]);
    }

    float operator[](int i) const
    {
      assert(i >= 0 && i <= 2);
      return vec[i];
    }

    float &operator[](int i)
    {
      assert(i >= 0 && i <= 2);
      return vec[i];
    }

    inline const float& x() const
    {
      return vec[0];
    }

    inline float& x()
    {
      return vec[0];
    }

    inline const float& y() const
    {
      return vec[1];
    }

    inline float& y()
    {
      return vec[1];
    }

    inline const float& z() const
    {
      return vec[2];
    }

    inline float& z()
    {
      return vec[2];
    }

    float LengthSquared() const
    {
      return vec.squaredNorm();
    }

    float Length() const
    {
      return vec.norm();
    }

    explicit Vector3f(const Normal3f &n);

    bool operator==(const Vector3f &v) const
    {
      return vec == v.vec;
    }

    bool operator!=(const Vector3f &v) const
    {
      return vec != v.vec;
    }

    friend std::ostream& operator<<(std::ostream& out, const Vector3f& v)
    {
      out.width(4);
      out.precision(3);
      out << v.vec[0] << ", " << v.vec[1] << ", " << v.vec[2];
      return out;
    }

    // Vector3f Public Data
    Eigen::Vector4f vec;
//    float& x, &y, &z;
};

struct Point3f
{
  public:
    // Point3f Public Methods
    Point3f()
        : vec(0.f, 0.f, 0.f, 1.f) //, x(vec[0]), y(vec[1]), z(vec[2])
    {

    }

    Point3f(float xx, float yy, float zz)
        : vec(xx, yy, zz, 1.f) //, x(vec[0]), y(vec[1]), z(vec[2])
    {
      assert(!HasNaNs());
    }

    Point3f(const Point3f& p)
        : vec(p.vec[0], p.vec[1], p.vec[2], 1.0f) //, x(vec[0]), y(vec[1]), z(vec[2])
    {
      assert(!p.HasNaNs());
    }

    Point3f& operator=(const Point3f &p)
    {
      assert(!p.HasNaNs());

      vec = p.vec;
      vec[3] = 1.0;

      return *this;
    }

    Point3f operator+(const Vector3f& v) const
    {
      assert(!v.HasNaNs());
      Eigen::Vector4f res = vec + v.vec;

      return Point3f(res[0], res[1], res[2]);
    }

    Point3f& operator+=(const Vector3f &v)
    {
      assert(!v.HasNaNs());
      vec += v.vec;
      vec[3] = 1.0f;

      return *this;
    }

    Vector3f operator-(const Point3f &p) const
    {
      assert(!p.HasNaNs());
      Eigen::Vector4f res = vec - p.vec;

      return Vector3f(res[0], res[1], res[2]);
    }

    Point3f operator-(const Vector3f& v) const
    {
      assert(!v.HasNaNs());
      Eigen::Vector4f res = vec - v.vec;

      return Point3f(res[0], res[1], res[2]);
    }

    Point3f& operator-=(const Vector3f& v)
    {
      assert(!v.HasNaNs());

      vec -= v.vec;
      vec[3] = 1.0f;

      return *this;
    }

    Point3f& operator+=(const Point3f& p)
    {
      assert(!p.HasNaNs());

      vec += p.vec;
      vec[3] = 1.0f;

      return *this;
    }

    Point3f operator+(const Point3f& p) const
    {
      assert(!p.HasNaNs());

      Eigen::Vector4f res = vec + p.vec;
      return Point3f(res[0], res[1], res[2]);
    }

    Point3f operator*(float f) const
    {
      Eigen::Vector4f res = f * vec;

      return Point3f(res[0], res[1], res[2]);
    }

    Point3f& operator*=(float f)
    {
      vec *= f;
      vec[3] = 1.0f;

      return *this;
    }

    Point3f operator/(float f) const
    {
      assert(f != 0);

      float inv = 1.f / f;
      Eigen::Vector4f res = vec * inv;

      return Point3f(res[0], res[1], res[2]);
    }

    Point3f& operator/=(float f)
    {
      assert(f != 0);
      float inv = 1.f / f;

      vec *= inv;
      vec[3] = 1.0f;

      return *this;
    }

    float operator[](int i) const
    {
      assert(i >= 0 && i <= 2);
      return vec[i];
    }

    float &operator[](int i)
    {
      assert(i >= 0 && i <= 2);
      return vec[i];
    }

    inline const float& x() const
    {
      return vec[0];
    }

    inline float& x()
    {
      return vec[0];
    }

    inline const float& y() const
    {
      return vec[1];
    }

    inline float& y()
    {
      return vec[1];
    }

    inline const float& z() const
    {
      return vec[2];
    }

    inline float& z()
    {
      return vec[2];
    }

    bool HasNaNs() const
    {
      return std::isnan(vec[0]) || std::isnan(vec[1]) || std::isnan(vec[2]);
    }

    bool operator==(const Point3f &p) const
    {

      return vec == p.vec;
    }

    bool operator!=(const Point3f &p) const
    {
      return vec != p.vec;
    }

    friend std::ostream& operator<<(std::ostream& out, const Point3f& p)
    {
      out.width(4);
      out.precision(3);
      out << p.vec[0] << ", " << p.vec[1] << ", " << p.vec[2];
      return out;
    }
    // Point3f Public Data
    Eigen::Vector4f vec;
//    float& x, &y, &z;
};

struct Normal3f
{
  public:
    // Normal3f Public Methods
    Normal3f()
        : vec(0.0f, 0.0f, 0.0f, 0.0f) //, x(vec[0]), y(vec[1]), z(vec[2])
    {

    }

    Normal3f(float xx, float yy, float zz)
        : vec(xx, yy, zz, 0.0f) //, x(vec[0]), y(vec[1]), z(vec[2])
    {
      assert(!HasNaNs());
    }

    Normal3f operator-() const
    {
      return Normal3f(-vec[0], -vec[1], -vec[2]);
    }

    Normal3f operator+(const Normal3f& n) const
    {
      assert(!n.HasNaNs());

      Eigen::Vector4f res = vec + n.vec;
      return Normal3f(res[0], res[1], res[2]);
    }

    Normal3f& operator+=(const Normal3f& n)
    {
      assert(!n.HasNaNs());

      vec += n.vec;

      return *this;
    }

    Normal3f operator-(const Normal3f& n) const
    {
      assert(!n.HasNaNs());

      Eigen::Vector4f res = vec - n.vec;
      return Normal3f(res[0], res[1], res[2]);
    }

    Normal3f& operator-=(const Normal3f& n)
    {
      assert(!n.HasNaNs());

      vec -= n.vec; // n.w == 0!

      return *this;
    }

    bool HasNaNs() const
    {
      return std::isnan(vec[0]) || std::isnan(vec[1]) || std::isnan(vec[2]);
    }

    Normal3f operator*(float f) const
    {
      Eigen::Vector4f res = f * vec;

      return Normal3f(res[0], res[1], res[2]);
    }

    Normal3f &operator*=(float f)
    {
      vec *= f; // vec.w == 0

      return *this;
    }

    Normal3f operator/(float f) const
    {
      assert(f != 0);
      float inv = 1.f / f;
      Eigen::Vector4f res = inv * vec;

      return Normal3f(res[0], res[1], res[2]);
    }

    Normal3f &operator/=(float f)
    {
      assert(f != 0);
      float inv = 1.f / f;
      vec *= inv;

      return *this;
    }

    float LengthSquared() const
    {
      return vec.squaredNorm();
    }

    float Length() const
    {
      return vec.norm();
    }

    Normal3f(const Normal3f &n)
        : vec(n.vec[0], n.vec[1], n.vec[2], 0.0f) //, x(vec[0]), y(vec[1]), z(vec[2])
    {
      assert(!n.HasNaNs());
    }

    Normal3f &operator=(const Normal3f &n)
    {
      assert(!n.HasNaNs());
      vec = n.vec;

      return *this;
    }

    explicit Normal3f(const Vector3f &v)
        : vec(v.vec[0], v.vec[1], v.vec[2], 0.0f) //, x(vec[0]), y(vec[1]), z(vec[2])
    {
      assert(!v.HasNaNs());
    }

    float operator[](int i) const
    {
      assert(i >= 0 && i <= 2);
      return vec[i];
    }

    float &operator[](int i)
    {
      assert(i >= 0 && i <= 2);
      return vec[i];
    }

    inline const float& x() const
    {
      return vec[0];
    }

    inline float& x()
    {
      return vec[0];
    }

    inline const float& y() const
    {
      return vec[1];
    }

    inline float& y()
    {
      return vec[1];
    }

    inline const float& z() const
    {
      return vec[2];
    }

    inline float& z()
    {
      return vec[2];
    }

    bool operator==(const Normal3f &n) const
    {
      return vec == n.vec;
    }
    bool operator!=(const Normal3f &n) const
    {
      return vec != n.vec;
    }

    // Normal3f Public Data
    Eigen::Vector4f vec;
//    float& x, &y, &z;
};

struct Ray
{
  public:
    // Ray Public Methods
    Ray()
        : mint(0.f), maxt(INFINITY), time(0.f), depth(0)
    {
    }

    Ray(const Point3f &origin, const Vector3f &direction, float start, float end = INFINITY, float t = 0.f, int d = 0)
        : o(origin), d(direction), mint(start), maxt(end), time(t), depth(d)
    {
    }

    Ray(const Point3f &origin, const Vector3f &direction, const Ray &parent, float start, float end = INFINITY)
        : o(origin), d(direction), mint(start), maxt(end), time(parent.time), depth(parent.depth + 1)
    {
    }

    Point3f operator()(float t) const
    {
      return o + d * t;
    }
    bool HasNaNs() const
    {
      return (o.HasNaNs() || d.HasNaNs() || std::isnan(mint) || std::isnan(maxt));
    }

    // Ray Public Data
    Point3f o;
    Vector3f d;
    mutable float mint, maxt;
    float time;
    int depth;
};

// Geometry Inline Functions
inline Vector3f::Vector3f(const Point3f& p)
    : vec(p.vec[0], p.vec[1], p.vec[2], 0.0f) //, x(vec[0]), y(vec[1]), z(vec[2])
{
  assert(!HasNaNs());
}

inline Vector3f operator*(float f, const Vector3f& v)
{
  return v * f;
}

inline float Dot(const Vector3f& v1, const Vector3f& v2)
{
  assert(!v1.HasNaNs() && !v2.HasNaNs());

  return v1.vec.dot(v2.vec) - v1.vec[3] * v2.vec[3]; // v1.w == v2.w == 1.0!
}

inline float AbsDot(const Vector3f& v1, const Vector3f& v2)
{
  assert(!v1.HasNaNs() && !v2.HasNaNs());

  return fabsf(v1.vec.dot(v2.vec) - v1.vec[3] * v2.vec[3]);
}

inline Vector3f Cross(const Vector3f& v1, const Vector3f& v2)
{
  assert(!v1.HasNaNs() && !v2.HasNaNs());
  Eigen::Vector4f res = v1.vec.cross3(v2.vec);

  return Vector3f(res[0], res[1], res[2]);
}

inline Vector3f Cross(const Vector3f &v1, const Normal3f &v2)
{
  assert(!v1.HasNaNs() && !v2.HasNaNs());
  Eigen::Vector4f res = v1.vec.cross3(v2.vec);

  return Vector3f(res[0], res[1], res[2]);
}

inline Vector3f Cross(const Normal3f &v1, const Vector3f &v2)
{
  assert(!v1.HasNaNs() && !v2.HasNaNs());
  Eigen::Vector4f res = v1.vec.cross3(v2.vec);

  return Vector3f(res[0], res[1], res[2]);
}

inline Vector3f Normalize(const Vector3f &v)
{
  return v / v.Length();
}

//inline void CoordinateSystem(const Vector3f &v1, Vector3f *v2, Vector3f *v3)
//{
//  if (fabsf(v1.vec[0]) > fabsf(v1.vec[1]))
//  {
//    float invLen = 1.f / sqrtf(v1.vec[0] * v1.vec[0] + v1.vec[2] * v1.vec[2]);
//    *v2 = Vector3f(-v1.vec[2] * invLen, 0.f, v1.vec[0] * invLen);
//  }
//  else
//  {
//    float invLen = 1.f / sqrtf(v1.vec[1] * v1.vec[1] + v1.vec[2] * v1.vec[2]);
//    *v2 = Vector3f(0.f, v1.vec[2] * invLen, -v1.vec[1] * invLen);
//  }
//  *v3 = Cross(v1, *v2);
//}

inline float Distance(const Point3f& p1, const Point3f& p2)
{
  return (p1 - p2).Length();
}

inline float DistanceSquared(const Point3f& p1, const Point3f& p2)
{
  return (p1 - p2).LengthSquared();
}

inline Point3f operator*(float f, const Point3f &p)
{
  assert(!p.HasNaNs());
  return p * f;
}

inline Normal3f operator*(float f, const Normal3f &n)
{
  return Normal3f(f * n.vec[0], f * n.vec[1], f * n.vec[2]);
}

inline Normal3f Normalize(const Normal3f& n)
{
  return n / n.Length();
}

inline Vector3f::Vector3f(const Normal3f& n)
    : vec(n.vec[0], n.vec[1], n.vec[2], 0.0f) //, x(vec[0]), y(vec[1]), z(vec[2])
{
  assert(!n.HasNaNs());
}

inline float Dot(const Normal3f& n1, const Vector3f& v2)
{
  assert(!n1.HasNaNs() && !v2.HasNaNs());

  return n1.vec.dot(v2.vec); // n1.w == 0!
}

inline float Dot(const Vector3f& v1, const Normal3f& n2)
{
  assert(!v1.HasNaNs() && !n2.HasNaNs());

  return v1.vec.dot(n2.vec); // n2.w == 0!
}

inline float Dot(const Normal3f &n1, const Normal3f &n2)
{
  assert(!n1.HasNaNs() && !n2.HasNaNs());

  return n1.vec.dot(n2.vec); // n2.w n == n1.w == 0!
}

inline float AbsDot(const Normal3f &n1, const Vector3f &v2)
{
  assert(!n1.HasNaNs() && !v2.HasNaNs());

  return fabsf(n1.vec.dot(v2.vec)); // n1.w == 0
}

inline float AbsDot(const Vector3f &v1, const Normal3f &n2)
{
  assert(!v1.HasNaNs() && !n2.HasNaNs());

  return fabsf(v1.vec.dot(n2.vec)); // n2.w == 0
}

inline float AbsDot(const Normal3f &n1, const Normal3f &n2)
{
  assert(!n1.HasNaNs() && !n2.HasNaNs());

  return fabsf(n1.vec.dot(n2.vec)); // n1.w == n2.w == 0
}

inline Normal3f Faceforward(const Normal3f& n, const Vector3f& v)
{
  return (Dot(n, v) < 0.f) ? -n : n;
}

inline Normal3f Faceforward(const Normal3f& n, const Normal3f& n2)
{
  return (Dot(n, n2) < 0.f) ? -n : n;
}

inline Vector3f Faceforward(const Vector3f& v, const Vector3f& v2)
{
  return (Dot(v, v2) < 0.f) ? -v : v;
}

inline Vector3f Faceforward(const Vector3f& v, const Normal3f& n2)
{
  return (Dot(v, n2) < 0.f) ? -v : v;
}

inline Vector3f SphericalDirection(float sintheta, float costheta, float phi)
{
  return Vector3f(sintheta * cosf(phi), sintheta * sinf(phi), costheta);
}

inline Vector3f SphericalDirection(float sintheta, float costheta, float phi, const Vector3f& x, const Vector3f& y,
    const Vector3f& z)
{
  return sintheta * cosf(phi) * x + sintheta * sinf(phi) * y + costheta * z;
}

inline float SphericalTheta(const Vector3f &v)
{
  return acosf(Clamp(v.vec[2], -1.f, 1.f));
}

inline float SphericalPhi(const Vector3f &v)
{
  float p = atan2f(v.vec[1], v.vec[0]);
  return (p < 0.f) ? p + 2.f * M_PI : p;
}

// traits:

namespace traits
{

template<>
struct access<rv::Point3f, 0>
{
    static float get(const rv::Point3f& p)
    {
      return p.vec[0];
    }

    static void set( rv::Point3f& p, float value)
    {
      p.vec[0] = value;
    }
};

template<>
struct access<rv::Point3f, 1>
{
    static float get(const rv::Point3f& p)
    {
      return p.vec[1];
    }

    static void set( rv::Point3f& p, float value)
    {
      p.vec[1] = value;
    }
};

template<>
struct access<rv::Point3f, 2>
{
    static float get(const rv::Point3f& p)
    {
      return p.vec[2];
    }

    static void set( rv::Point3f& p, float value)
    {
      p.vec[2] = value;
    }
};

template<>
struct access<rv::Normal3f, 0>
{
    static float get(const rv::Normal3f& n)
    {
      return n.vec[0];
    }

    static void set( rv::Normal3f& n, float value)
    {
      n.vec[0] = value;
    }
};

template<>
struct access<rv::Normal3f, 1>
{
    static float get(const rv::Normal3f& n)
    {
      return n.vec[1];
    }

    static void set( rv::Normal3f& n, float value)
    {
      n.vec[1] = value;
    }
};

template<>
struct access<rv::Normal3f, 2>
{
    static float get(const rv::Normal3f& n)
    {
      return n.vec[2];
    }

    static void set( rv::Normal3f& n, float value)
    {
      n.vec[2] = value;
    }
};

template<>
struct access<rv::Vector3f, 0>
{
    static float get(const rv::Vector3f& v)
    {
      return v.vec[0];
    }

    static void set(rv::Vector3f& v, float value)
    {
      v.vec[0] = value;
    }
};

template<>
struct access<rv::Vector3f, 1>
{
    static float get(const rv::Vector3f& v)
    {
      return v.vec[1];
    }

    static void set(rv::Vector3f& v, float value)
    {
      v.vec[1] = value;
    }
};

template<>
struct access<rv::Vector3f, 2>
{
    static float get(const rv::Vector3f& v)
    {
      return v.vec[2];
    }

    static void set(rv::Vector3f& v, float value)
    {
      v.vec[2] = value;
    }
};

}

}
#endif // GEOMETRY_H
