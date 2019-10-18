/*****************************************************************************\
 \headerfile Math.h

 \brief some math utility functions

 \author behley

 \*****************************************************************************/

#ifndef MATH_H_
#define MATH_H_

#include <string>
#include <vector>

#include <cmath>
#include <eigen3/Eigen/Dense>

namespace rv
{

namespace Math
{
const double PI = M_PI;

/** \brief store a given matrix in Octave compatible text format.
 *  \param filename  filename of the resulting text file.
 *  \param varname  name of the variable in Octave
 *  \param mat stored matrix
 **/
void save(const std::string& filename, const std::string& varname, const Eigen::MatrixXf& mat);
void save(const std::string& filename, const std::string& varname, const Eigen::MatrixXd& mat);

/** \brief loading a matrix from a given file. The format must be Octave compatible.
 *  \param filename filename of the matrix
 *  \param M matrix variable
 *
 *  \return true, if matrix could be initialized using the file information. false, otherwise.
 */
bool load(const std::string& filename, Eigen::MatrixXf& M);
bool load(const std::string& filename, Eigen::MatrixXd& M);

/** \brief convert an angle given in degrees to radian **/
inline double deg2rad(double deg)
{
  return deg * PI / 180.0;
}
/** \brief convert an angle given in radian to degrees **/
inline double rad2deg(double rad)
{
  return rad * 180.0 / PI;
}

/** \brief rounding to the nearest integer. **/
inline double round(double a)
{
  return std::floor(a + 0.5);
}

/** \brief normalizes a given angle to [0, 2pi] **/
inline float normalizeAngle(float angle)
{
  while (angle > 2.0f * PI)
    angle -= 2.0f * PI;
  while (angle < 0)
    angle += 2.0f * PI;

  return angle;
}

/**
 * \brief Covariance Matrix [MatLab equivalent of cov(x)]
 * For matrices, where each row is an observation, and each column is a variable, cov(X) is the covariance matrix.
 *
 * diag(cov(X)) is a vector of variances for each column, and sqrt(diag(cov(X))) is a vector of standard deviations.
 *
 * cov(x) normalizes by N-1, if N>1, where N is the number of observations.
 *
 * This makes cov(X) the best unbiased estimate of the covariance matrix if the observations are from a
 * normal distribution. For N=1, cov normalizes by N.
 *
 * cov(x,1) normalizes by N and produces the second moment matrix of the observations about their mean.
 * cov(X,0) is the same as cov(X).
 */
Eigen::MatrixXf cov(const Eigen::MatrixXf& X, const int32_t N = 0);

/** \brief helper function to calculate two dimensional covariance matrix of three-dimensional points.
 **/
template<class PointIterator>
Eigen::Matrix2f cov2(PointIterator first, PointIterator last)
{
  Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
  Eigen::Vector2f mean = Eigen::Vector2f::Zero();
  Eigen::Vector2f x;
  uint32_t N = 0;

  for (PointIterator it = first; it != last; ++it, ++N)
  {
    x(0, 0) = it->x();
    x(1, 0) = it->y();
    cov += x * x.transpose();
    mean += x;
  }

  mean *= 1.0 / N;

  /** calculate eigenvectors in x,y direction, and set bounding box. **/
  cov = 1.0 / N * cov - mean * mean.transpose();

  return cov;
}

/** \brief helper function to calculate three-dimensional covariance matrix of three-dimensional points.
 **/
template<class PointIterator>
Eigen::Matrix3f cov3(PointIterator first, PointIterator last)
{
  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
  Eigen::Vector3f mean = Eigen::Vector3f::Zero();
  Eigen::Vector3f x;

  uint32_t N = 0;

  for (PointIterator it = first; it != last; ++it, ++N)
  {
    x(0, 0) = it->x();
    x(1, 0) = it->y();
    x(2, 0) = it->z();
    cov += x * x.transpose();
    mean += x;
  }

  mean *= 1.0 / N;

  cov = 1.0 / N * cov - mean * mean.transpose();

  return cov;
}

/**
 * \brief smallest elements in array
 *
 * C = min(A) returns the smallest elements along different dimensions of an array.
 *
 * If A is a vector, min(A) returns the smallest element in A.
 *
 * If A is a matrix, min(A) treats the columns of A as vectors,
 * returning a row vector containing the minimum element from each column.
 *
 */

double min(const Eigen::VectorXf& V);
Eigen::MatrixXf min(const Eigen::MatrixXf& M);
/**
 * \brief largest elements in array
 *
 * C = max(A) returns the largest elements along different dimensions of an array.
 *
 * If A is a vector, max(A) returns the largest element in A.
 *
 * If A is a matrix, max(A) treats the columns of A as vectors,
 * returning a row vector containing the maximum element from each column.
 *
 */

double max(const Eigen::VectorXf& V);
Eigen::MatrixXf max(const Eigen::MatrixXf& M);
/**
 * \brief sorts the elements along different dimensions of an array,
 * and arranges those elements in ascending order
 *
 * (NOT IMPLEMENTED) B = sort(A,dim) sorts the elements along the dimension of A specified by a scalar dim.
 *
 * (NOT IMPLEMENTED) B = sort(...,mode) sorts the elements in the specified direction, depending on the value of mode.
 *
 * 'ascend'           Ascending order (default)
 * 'descend'          Descending order
 *
 * sort(A,B,IX) also returns an array of indices IX, where size(IX) == size(A).
 * If A is a vector, B = A(IX).  If A is an m-by-n matrix, then each column of IX is a permutation
 * vector of the corresponding column of A, such that
 * for j = 1:n
 *    B(:,j) = A(IX(:,j),j);
 * end
 *
 *
 */
void sort(const Eigen::MatrixXf& M, Eigen::MatrixXf& result, Eigen::MatrixXf& indices);

/**
 * \brief Product of array elements
 *
 * B = prod(A) returns the products along different dimensions of an array.
 *
 * (NOT IMPLEMENTED) If A is a vector, prod(A) returns the product of the elements.
 *
 * If A is a matrix, prod(A) treats the columns of A as vectors, returning a row vector of the products of each column.
 *
 * B = prod(A,dim) takes the products along the dimension of A specified by scalar dim.
 */
Eigen::VectorXf prod(const Eigen::MatrixXf& M, int dim = 0);

/**
 * \brief Load data from disk file
 *  importdata(filename) loads data from filename into the workspace. The filename input is a string enclosed
 *  in single quotes.
 *
 *  A = importdata(filename) loads data from filename into structure A.
 *
 *  A = importdata(filename,delimiter) loads data from filename using delimiter as the column separator.
 *  The delimiter argument must be a string enclosed in single quotes. Use '\t' for tab.
 *  When importing from an ASCII file, delimiter only separates numeric data.
 *
 *  A = importdata(filename,delimiter,headerline) where headerline is a number that indicates on
 *  which line of the file the header text is located, loads data from line headerline+1
 *  to the end of the file. (The first line is indexed by 1)
 *
 *  Currently only text files are supported.
 */
//TODO: legacy_fkie::Matrix importdata(const std::string& filename, const std::string& delimiter = " ",
//    const unsigned int headerline = 0);
/**
 * \brief calculates the square-root of a symmetric and positive semi-definite matrix M \in R^(n x n), which satisfies M = M^(1/2)*M^(1/2)
 *
 *  If M is diagonizable that means M = Z*D*Z', where Z is orthogonal (i.e. Id = Z*Z^T = Z^T*Z)
 *  and D is diagonal  than M^(1/2) is given by Z*D^(1/2)*Z^T. (Note, that
 *  D^(1/2) is given by the square root of the diagonal elements.)
 *
 *  Thus, we can get M^(1/2) by the eigenvectors and eigenvalues of M.
 */
Eigen::MatrixXf sqrt(const Eigen::MatrixXf& M);
/**
 * \brief returns a randomized permutation of integers up to n
 *
 */
//legacy_fkie::Vector randperm(const unsigned int n);
/** \brief calculate the square of a value. **/
template<typename T>
inline T sqr(T val)
{
  return val * val;
}

/** \brief get the minimum of a vector **/
template<typename T>
T min(const std::vector<T>& vec)
{
  T minimum = vec[0];

  for (uint32_t i = 1; i < vec.size(); ++i)
  {
    if (vec[i] < minimum) minimum = vec[i];
  }

  return minimum;
}

/** \brief get the maximum of a vector **/
template<typename T>
T max(const std::vector<T>& vec)
{
  T maximum = vec[0];

  for (uint32_t i = 1; i < vec.size(); ++i)
  {
    if (vec[i] > maximum) maximum = vec[i];
  }

  return maximum;
}

/** \brief calculate mean of iteratable list **/
template<class ForwardIterator>
double mean(ForwardIterator first, ForwardIterator last)
{
  double mean = 0.0;
  uint32_t size = 0;

  for (ForwardIterator it = first; it != last; ++it, ++size)
    mean += *it;

  return mean / double(size);
}

/** \brief calculate variance of iterateable list **/
template<class ForwardIterator>
double sigma(ForwardIterator first, ForwardIterator last, const int N = 0)
{
  double xx = 0.0;
  double mean = 0.0;
  uint32_t size = 0;

  for (ForwardIterator it = first; it != last; ++it, ++size)
  {
    xx += (*it) * (*it);
    mean += *it;
  }

  mean /= double(size);

  return std::sqrt(1.0 / double(size - 1 + N) * xx - double(size) / double(size - 1 + N) * mean * mean);
}

/** \brief compute sum of a sequence of numbers. **/
template<class ForwardIterator>
double sum(ForwardIterator first, ForwardIterator last)
{
  double sum = 0.0;
  for (ForwardIterator it = first; it != last; ++it)
    sum += *it;

  return sum;
}

/** \brief sum over a vector of values **/
template<typename T>
T sum(const std::vector<T>& vec)
{
  T sum = vec[0];
  for (uint32_t i = 1; i < vec.size(); ++i)
    sum += vec[i];

  return sum;
}

/** \brief replicate and tile an array.
 *
 * B = repmat(A,m,n) creates a large matrix B consisting of an m-by-n tiling of copies of A.
 * The size of B is [A.rows()*m, (A.cols()*n].
 * The statement repmat(A,n) creates an n-by-n tiling.
 * **/

//template<typename T>
//Eigen::Matrix<T> repmat(const Eigen::Matrix<T>&, uint32_t m, uint32_t n = m)
//{
//
//}
/**
 * \brief normal pdf with mu and sigma.
 *
 * @param x
 * @param mu expected value of the pdf
 * @param sigma standard deviation of the normal distribution
 *
 * @return value of the normal pdf at x with mu and sigma.
 */
double normal_pdf(double x, double mu = 0.0, double sigma = 1.0);

double exponential_pdf(double x, double lambda = 1.0);

/**
 * \brief reparameterized sigmoid function.
 *
 * loc defines, where the sigmoid takes value 0.5.
 * scale specifies the steepness of the function around loc.
 *
 * @param x
 * @param loc  location of sigmoid(x) = 0.5
 * @param scale "steepness" of sigmoid function around loc.
 *
 * @return value in [0,1], where sigmoid(loc) = 0.5.
 */
double sigmoid2(double x, double loc = 0.0, double scale = 1.0);

template<typename T>
uint32_t argmax(const std::vector<T>& vec)
{
  uint32_t maxidx = 0;

  for (uint32_t i = 1; i < vec.size(); ++i)
    if (vec[maxidx] < vec[i]) maxidx = i;

  return maxidx;
}

Eigen::Matrix2f diag(float val0, float val1);
Eigen::Matrix3f diag(float val0, float val1, float val2);

}

}
#endif /* MATH_H_ */
