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

// core/transform.cpp*
#include <rv/transform.h>
//#include <legacy_fkie/MathError.h>
#include <cstring>

namespace rv
{

//#define IDX(i,j) (i)+4*(j)
//
//// Matrix4x4 Method Definitions
//bool SolveLinearSystem2x2(const float A[2][2], const float B[2], float *x0,
//    float *x1)
//{
//  float det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
//  if (fabsf(det) < 1e-10f) return false;
//  *x0 = (A[1][1] * B[0] - A[0][1] * B[1]) / det;
//  *x1 = (A[0][0] * B[1] - A[1][0] * B[0]) / det;
//  if (isnan(*x0) || isnan(*x1)) return false;
//  return true;
//}
//
//Matrix4x4::Matrix4x4(float mat[4][4])
//{
//  /**memcpy(m, mat, 16*sizeof(float));**/
//  m[IDX(0,0)] = mat[0][0];
//  m[IDX(0,1)] = mat[0][1];
//  m[IDX(0,2)] = mat[0][2];
//  m[IDX(0,3)] = mat[0][3];
//  m[IDX(1,0)] = mat[1][0];
//  m[IDX(1,1)] = mat[1][1];
//  m[IDX(1,2)] = mat[1][2];
//  m[IDX(1,3)] = mat[1][3];
//  m[IDX(2,0)] = mat[2][0];
//  m[IDX(2,1)] = mat[2][1];
//  m[IDX(2,2)] = mat[2][2];
//  m[IDX(2,3)] = mat[2][3];
//  m[IDX(3,0)] = mat[3][0];
//  m[IDX(3,1)] = mat[3][1];
//  m[IDX(3,2)] = mat[3][2];
//  m[IDX(3,3)] = mat[3][3];
//}
//
//Matrix4x4::Matrix4x4(float t00, float t01, float t02, float t03, float t10,
//    float t11, float t12, float t13, float t20, float t21, float t22, float t23,
//    float t30, float t31, float t32, float t33)
//{
//  m[IDX(0,0)] = t00;
//  m[IDX(0,1)] = t01;
//  m[IDX(0,2)] = t02;
//  m[IDX(0,3)] = t03;
//  m[IDX(1,0)] = t10;
//  m[IDX(1,1)] = t11;
//  m[IDX(1,2)] = t12;
//  m[IDX(1,3)] = t13;
//  m[IDX(2,0)] = t20;
//  m[IDX(2,1)] = t21;
//  m[IDX(2,2)] = t22;
//  m[IDX(2,3)] = t23;
//  m[IDX(3,0)] = t30;
//  m[IDX(3,1)] = t31;
//  m[IDX(3,2)] = t32;
//  m[IDX(3,3)] = t33;
//}
//
//Matrix4x4 Transpose(const Matrix4x4 &m)
//{
//  return Matrix4x4(m(0, 0), m(1, 0), m(2, 0), m(3, 0), m(0, 1), m(1, 1),
//      m(2, 1), m(3, 1), m(0, 2), m(1, 2), m(2, 2), m(3, 2), m(0, 3), m(1, 3),
//      m(2, 3), m(3, 3));
//}
//
//Matrix4x4 Inverse(const Matrix4x4 &m)
//{
//  int indxc[4], indxr[4];
//  int ipiv[4] =
//  { 0, 0, 0, 0 };
//  float minv[4][4];
//  //    memcpy(minv, m.m, 4*4*sizeof(float));
//  /** copy from column-major to row-major **/
//  minv[0][0] = m.m[0];
//  minv[1][0] = m.m[1];
//  minv[2][0] = m.m[2];
//  minv[3][0] = m.m[3];
//  minv[0][1] = m.m[4];
//  minv[1][1] = m.m[5];
//  minv[2][1] = m.m[6];
//  minv[3][1] = m.m[7];
//  minv[0][2] = m.m[8];
//  minv[1][2] = m.m[9];
//  minv[2][2] = m.m[10];
//  minv[3][2] = m.m[11];
//  minv[0][3] = m.m[12];
//  minv[1][3] = m.m[13];
//  minv[2][3] = m.m[14];
//  minv[3][3] = m.m[15];
//
//  for (int i = 0; i < 4; i++)
//  {
//    int irow = -1, icol = -1;
//    float big = 0.;
//    // Choose pivot
//    for (int j = 0; j < 4; j++)
//    {
//      if (ipiv[j] != 1)
//      {
//        for (int k = 0; k < 4; k++)
//        {
//          if (ipiv[k] == 0)
//          {
//            if (fabsf(minv[j][k]) >= big)
//            {
//              big = float(fabsf(minv[j][k]));
//              irow = j;
//              icol = k;
//            }
//          }
//          else if (ipiv[k] > 1) throw RoSe::MathError(
//              "Singular matrix in MatrixInvert");
//        }
//      }
//    }
//    ++ipiv[icol];
//    // Swap rows _irow_ and _icol_ for pivot
//    if (irow != icol)
//    {
//      for (int k = 0; k < 4; ++k)
//        std::swap(minv[irow][k], minv[icol][k]);
//    }
//    indxr[i] = irow;
//    indxc[i] = icol;
//    if (minv[icol][icol] == 0.) throw RoSe::MathError(
//        "Singular matrix in MatrixInvert");
//
//    // Set $m[icol][icol]$ to one by scaling row _icol_ appropriately
//    float pivinv = 1.f / minv[icol][icol];
//    minv[icol][icol] = 1.f;
//    for (int j = 0; j < 4; j++)
//      minv[icol][j] *= pivinv;
//
//    // Subtract this row from others to zero out their columns
//    for (int j = 0; j < 4; j++)
//    {
//      if (j != icol)
//      {
//        float save = minv[j][icol];
//        minv[j][icol] = 0;
//        for (int k = 0; k < 4; k++)
//          minv[j][k] -= minv[icol][k] * save;
//      }
//    }
//  }
//  // Swap columns to reflect permutation
//  for (int j = 3; j >= 0; j--)
//  {
//    if (indxr[j] != indxc[j])
//    {
//      for (int k = 0; k < 4; k++)
//        std::swap(minv[k][indxr[j]], minv[k][indxc[j]]);
//    }
//  }
//  return Matrix4x4(minv);
//}
//
//std::ostream& operator<<(std::ostream& out, const Matrix4x4& mat)
//{
//  std::ios_base::fmtflags old_flags = out.flags();
//  std::streamsize old_width = out.width();
//  std::streamsize old_precision = out.precision();
//
//  out.precision(4);
//  out.width(10);
//
//  out << std::right;
//
//  for (unsigned int i = 0; i < 3; ++i)
//  {
//    for (unsigned int j = 0; j < 3; ++j)
//    {
//      out.precision(4);
//      out.width(10);
//      out << mat(i, j) << " ";
//    }
//    out.precision(4);
//    out.width(10);
//    out << mat(i, 3) << std::endl;
//  }
//
//  for (unsigned int j = 0; j < 3; ++j)
//  {
//    out.precision(4);
//    out.width(10);
//    out << mat(3, j) << " ";
//  }
//
//  out.precision(4);
//  out.width(10);
//  out << mat(3, 3);
//
//  /** restore old formating options. **/
//  out.flags(old_flags);
//  out.width(old_width);
//  out.precision(old_precision);
//
//  return out;
//}

// Transform Method Definitions
//void Transform::Print(FILE *f) const
//{
//  m.Print(f);
//}

Vector3f Translation(const Transform& t)
{
  const Eigen::Matrix4f& m = t.GetMatrix();
  return Vector3f(m(0, 3), m(1, 3), m(2, 3));
}

Transform Translate(const Vector3f& delta)
{
  Eigen::Matrix4f m;
  m << 1, 0, 0, delta.x(), 0, 1, 0, delta.y(), 0, 0, 1, delta.z(), 0, 0, 0, 1;
  Eigen::Matrix4f minv;
  minv << 1, 0, 0, -delta.x(), 0, 1, 0, -delta.y(), 0, 0, 1, -delta.z(), 0, 0, 0, 1;

  return Transform(m, minv);
}

Transform Scale(float x, float y, float z)
{
  Eigen::Matrix4f m;
  m << x, 0, 0, 0, 0, y, 0, 0, 0, 0, z, 0, 0, 0, 0, 1;
  Eigen::Matrix4f minv;
  minv << 1.f / x, 0, 0, 0, 0, 1.f / y, 0, 0, 0, 0, 1.f / z, 0, 0, 0, 0, 1;

  return Transform(m, minv);
}

Transform RotateX(float angle)
{
  float sin_t = sinf(Radians(angle));
  float cos_t = cosf(Radians(angle));
  Eigen::Matrix4f m;
  m << 1, 0, 0, 0, 0, cos_t, -sin_t, 0, 0, sin_t, cos_t, 0, 0, 0, 0, 1;
  return Transform(m, m.transpose());
}

Transform RotateY(float angle)
{
  float sin_t = sinf(Radians(angle));
  float cos_t = cosf(Radians(angle));
  Eigen::Matrix4f m;
  m << cos_t, 0, sin_t, 0, 0, 1, 0, 0, -sin_t, 0, cos_t, 0, 0, 0, 0, 1;
  return Transform(m, m.transpose());
}

Transform RotateZ(float angle)
{
  float sin_t = sinf(Radians(angle));
  float cos_t = cosf(Radians(angle));
  Eigen::Matrix4f m;
  m << cos_t, -sin_t, 0, 0, sin_t, cos_t, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  return Transform(m, m.transpose());
}

Transform Rotate(float angle, const Vector3f& axis)
{
  Vector3f a = Normalize(axis);
  float s = sinf(Radians(angle));
  float c = cosf(Radians(angle));
  float m[4][4];

  m[0][0] = a.x() * a.x() + (1.f - a.x() * a.x()) * c;
  m[0][1] = a.x() * a.y() * (1.f - c) - a.z() * s;
  m[0][2] = a.x() * a.z() * (1.f - c) + a.y() * s;
  m[0][3] = 0;

  m[1][0] = a.x() * a.y() * (1.f - c) + a.z() * s;
  m[1][1] = a.y() * a.y() + (1.f - a.y() * a.y()) * c;
  m[1][2] = a.y() * a.z() * (1.f - c) - a.x() * s;
  m[1][3] = 0;

  m[2][0] = a.x() * a.z() * (1.f - c) - a.y() * s;
  m[2][1] = a.y() * a.z() * (1.f - c) + a.x() * s;
  m[2][2] = a.z() * a.z() + (1.f - a.z() * a.z()) * c;
  m[2][3] = 0;

  m[3][0] = 0;
  m[3][1] = 0;
  m[3][2] = 0;
  m[3][3] = 1;

  Eigen::Matrix4f mat;
  mat << m[0][0], m[0][1], m[0][2], m[0][3], m[1][0], m[1][1], m[1][2], m[1][3], m[2][0], m[2][1], m[2][2], m[2][3], m[3][0], m[3][1], m[3][2], m[3][3];
  return Transform(mat, mat.transpose());
}

Transform LookAt(const Point3f &pos, const Point3f &look, const Vector3f &up)
{
  float m[4][4];
  // Initialize fourth column of viewing matrix
  m[0][3] = pos.x();
  m[1][3] = pos.y();
  m[2][3] = pos.z();
  m[3][3] = 1;

  // Initialize first three columns of viewing matrix
  Vector3f dir = Normalize(look - pos);
  Vector3f left = Normalize(Cross(Normalize(up), dir));
  Vector3f newUp = Cross(dir, left);

  m[0][0] = left.x();
  m[1][0] = left.y();
  m[2][0] = left.z();
  m[3][0] = 0.;
  m[0][1] = newUp.x();
  m[1][1] = newUp.y();
  m[2][1] = newUp.z();
  m[3][1] = 0.;
  m[0][2] = dir.x();
  m[1][2] = dir.y();
  m[2][2] = dir.z();
  m[3][2] = 0.;

  Eigen::Matrix4f camToWorld;
  camToWorld << m[0][0], m[0][1], m[0][2], m[0][3], m[1][0], m[1][1], m[1][2], m[1][3], m[2][0], m[2][1], m[2][2], m[2][3], m[3][0], m[3][1], m[3][2], m[3][3];
  return Transform(camToWorld.inverse(), camToWorld);
}

//BBox Transform::operator()(const BBox &b) const {
//    const Transform &M = *this;
//    BBox ret(        M(Point(b.pMin.x(), b.pMin.y(), b.pMin.z())));
//    ret = Union(ret, M(Point(b.pMax.x(), b.pMin.y(), b.pMin.z())));
//    ret = Union(ret, M(Point(b.pMin.x(), b.pMax.y(), b.pMin.z())));
//    ret = Union(ret, M(Point(b.pMin.x(), b.pMin.y(), b.pMax.z())));
//    ret = Union(ret, M(Point(b.pMin.x(), b.pMax.y(), b.pMax.z())));
//    ret = Union(ret, M(Point(b.pMax.x(), b.pMax.y(), b.pMin.z())));
//    ret = Union(ret, M(Point(b.pMax.x(), b.pMin.y(), b.pMax.z())));
//    ret = Union(ret, M(Point(b.pMax.x(), b.pMax.y(), b.pMax.z())));
//    return ret;
//}

Transform Transform::operator*(const Transform &t2) const
{
  Eigen::Matrix4f m1 = m * t2.m;
  Eigen::Matrix4f m2 = t2.mInv * mInv;

  return Transform(m1, m2);
}

bool Transform::SwapsHandedness() const
{
  float det = ((m(0, 0) * (m(1, 1) * m(2, 2) - m(1, 2) * m(2, 1)))
      - (m(0, 1) * (m(1, 0) * m(2, 2) - m(1, 2) * m(2, 0)))
      + (m(0, 2) * (m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0))));
  return det < 0.f;
}

Transform Orthographic(float znear, float zfar)
{
  return Scale(1.f, 1.f, 1.f / (zfar - znear))
      * Translate(Vector3f(0.f, 0.f, -znear));
}

Transform Perspective(float fov, float n, float f)
{
  // Perform projective divide
  Eigen::Matrix4f persp;
  persp << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, f / (f - n), -f * n / (f - n), 0, 0, 1, 0;

  // Scale to canonical viewing volume
  float invTanAng = 1.f / tanf(Radians(fov) / 2.f);
  return Scale(invTanAng, invTanAng, 1) * Transform(persp);
}

Transform::Transform(float yaw, float pitch, float roll, const Vector3f& t,
    int convention)
{
  float s1 = sinf(yaw), c1 = cosf(yaw);
  float s2 = sinf(pitch), c2 = cosf(pitch);
  float s3 = sinf(roll), c3 = cosf(roll);

  m = Eigen::Matrix4f::Identity();
  mInv = Eigen::Matrix4f::Identity();

  if (convention == 0) /** RotateZ(yaw)*RotateY(pitch)*RotateX(roll) **/
  {
    m(0, 0) = c1 * c2;
    m(0, 1) = c1 * s2 * s3 - s1 * c3;
    m(0, 2) = s1 * s3 + c1 * s2 * c3;
    m(1, 0) = s1 * c2;
    m(1, 1) = s1 * s2 * s3 + c1 * c3;
    m(1, 2) = s1 * s2 * c3 - c1 * s3;
    m(2, 0) = -s2;
    m(2, 1) = c2 * s3;
    m(2, 2) = c2 * c3;
  }
  else
  {
    m(0, 0) = c1 * c2 - s1 * s2 * s3;
    m(0, 1) = -s1 * c3;
    m(0, 2) = c1 * s2 + s1 * s3 * c2;
    m(1, 0) = c2 * s1 + c1 * s2 * s3;
    m(1, 1) = c1 * c3;
    m(1, 2) = s1 * s2 - c1 * s3 * c2;
    m(2, 0) = -s2 * c3;
    m(2, 1) = s3;
    m(2, 2) = c3 * c2;
  }

  /** the inverted matrix is given by the transpose of m ... **/
  mInv = m.transpose();

  Vector3f t_inv(mInv(0, 0) * t.x() + mInv(0, 1) * t.y() + mInv(0, 2) * t.z(),
      mInv(1, 0) * t.x() + mInv(1, 1) * t.y() + mInv(1, 2) * t.z(),
      mInv(2, 0) * t.x() + mInv(2, 1) * t.y() + mInv(2, 2) * t.z());

  m(0, 3) = t.x();
  m(1, 3) = t.y();
  m(2, 3) = t.z();

  /** ... and the inverse translation **/
  mInv(0, 3) = -t_inv.x();
  mInv(1, 3) = -t_inv.y();
  mInv(2, 3) = -t_inv.z();
}

#undef IDX
}
