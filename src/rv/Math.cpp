#include "rv/Math.h"
#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <list>
#include <sstream>
#include "rv/IOError.h"
#include "rv/Random.h"
#include "rv/string_utils.h"

namespace rv {

namespace Math {

struct SortElement {
  SortElement(unsigned int ind, double val) : index(ind), value(val) {}

  bool operator<(const SortElement& other) { return (value < other.value); }

  uint32_t index;
  double value;
};

Eigen::MatrixXf cov(const Eigen::MatrixXf& X, const int32_t N) {
  int n = std::min(1, std::max(N, 0));
  if (X.rows() == 1) n = 1;

  // calculate the mean

  Eigen::VectorXf mean = X.colwise().sum();
  mean = mean / X.rows();

  Eigen::MatrixXf X_mean = Eigen::MatrixXf::Zero(X.rows(), X.cols());
  for (uint32_t i = 0; i < X.rows(); ++i)
    for (uint32_t j = 0; j < X.cols(); ++j) X_mean(i, j) = X(i, j) - mean[j];

  Eigen::MatrixXf covariance = 1.0 / (double)(X.rows() - 1 + n) * X_mean.transpose() * X_mean;

  return covariance;
}

// double min(const legacy_fkie::Vector& V)
//{
//  return V.min();
//}
//

Eigen::MatrixXf min(const Eigen::MatrixXf& M) { return M.colwise().minCoeff(); }

//
// double max(const legacy_fkie::Vector& V)
//{
//  return V.max();
//}
//
Eigen::MatrixXf max(const Eigen::MatrixXf& M) { return M.colwise().maxCoeff(); }

void sort(const Eigen::MatrixXf& M, Eigen::MatrixXf& result, Eigen::MatrixXf& indices) {
  std::list<SortElement> temp;

  for (uint32_t c = 0; c < M.cols(); ++c) {
    for (uint32_t r = 0; r < M.rows(); ++r) temp.push_back(SortElement(r, M(r, c)));

    temp.sort();

    uint32_t r = 0;
    while (!temp.empty()) {
      indices(r, c) = temp.front().index;
      result(r, c) = temp.front().value;
      temp.pop_front();  // clear the list
      ++r;
    }
  }
}
//
// legacy_fkie::Vector prod(const legacy_fkie::Matrix& M, int dim)
//{
//  legacy_fkie::Vector result;
//  if (dim == 0)
//  {
//    result = legacy_fkie::Vector(M.cols());
//    for (unsigned int c = 0; c < M.cols(); ++c)
//    {
//      double product = 1.0;
//      for (unsigned int r = 0; r < M.rows(); ++r)
//        product *= M(r, c);
//      result[c] = product;
//    }
//  }
//  else if (dim == 1)
//  {
//    result = legacy_fkie::Vector(M.rows());
//
//    for (unsigned int r = 0; r < M.rows(); ++r)
//    {
//      double product = 1.0;
//      for (unsigned int c = 0; c < M.cols(); ++c)
//        product *= M(r, c);
//      result[r] = product;
//    }
//  }
//
//  return result;
//}
//
// legacy_fkie::Matrix importdata(const std::string& filename,
//    const std::string& delimiter, const unsigned int headerline)
//{
//  const unsigned int MAX_LENGTH = 64000;
//  char line[MAX_LENGTH];
//  memset(line, 0, MAX_LENGTH * sizeof(char));
//  char* token = 0;
//
//  std::ifstream file(filename.c_str());
//  if (!file.good()) throw legacy_fkie::IOError(
//      "importdata: failed opening " + filename);
//
//  // determine the rows and column count
//  unsigned int lineno = 0;
//  unsigned int rows = 0;
//  unsigned int cols = 0;
//  while (!file.eof())
//  {
//    file.getline(line, 64000);
//    ++lineno;
//    if (lineno <= headerline) continue;
//
//    // count columns
//    if (rows == 0)
//    {
//      token = strtok(line, delimiter.c_str());
//      while (token != 0)
//      {
//        ++cols;
//        token = strtok(0, delimiter.c_str());
//      }
//    }
//    ++rows;
//  }
//  file.close();
//
//  legacy_fkie::Matrix result(rows, cols);
//  file.open(filename.c_str());
//  unsigned int r = 0;
//  unsigned int c = 0;
//  lineno = 0;
//  while (!file.eof())
//  {
//    c = 0;
//    file.getline(line, 64000);
//    ++lineno;
//    if (lineno <= headerline) continue;
//    // split line
//    token = strtok(line, delimiter.c_str());
//    while (token != 0)
//    {
//      result(r, c) = atof(token);
//      token = strtok(0, delimiter.c_str());
//      ++c;
//    }
//    ++r;
//  }
//
//  file.close();
//
//  return result;
//}
//
// legacy_fkie::Matrix sqrt(const legacy_fkie::Matrix& M)
//{
//  legacy_fkie::Matrix Z = M;
//  std::vector<double> ev;
//
//  int info = legacy_fkie::LA::dsyev('V', 'U', Z, ev);
//  if (info < 0)
//  {
//    throw legacy_fkie::MathError(
//        "failed to determine the eigenvalues and eigenvectors of M");
//  }
//  else if (info > 0)
//  {
//    throw legacy_fkie::MathError("failed to converge!");
//  }
//
////  std::cout << ev.size() << std::endl;
//  /* calculate D^(1/2) */
//  legacy_fkie::Matrix D(ev.size(), ev.size());
//  for (unsigned int i = 0; i < ev.size(); ++i)
//  {
//    // epsilon test for values near zero.
//    if (std::abs(ev[i]) < 0.0000001)
//    {
//      D(i, i) = 0.0;
//      continue;
//    }
//    if (ev[i] < 0)
//    {
//      std::stringstream sstr;
//      sstr
//          << "Matrix M is not positive semi-definite. Eigenvalue less than zero: "
//          << ev[i];
//      throw legacy_fkie::MathError(sstr.str());
//    }
//    D(i, i) = std::sqrt(ev[i]);
//  }
//
//  return Z * D * Z.transpose();
//}

// legacy_fkie::Vector randperm(const unsigned int n)
//{
//  static legacy_fkie::Random rand;
//  legacy_fkie::Vector v(n);
//
//  for (unsigned int i = 1; i <= n; ++i)
//    v[i - 1] = i;
//
//  std::random_shuffle(v.ptr(), v.ptr() + n, rand);
//
//  return v;
//}

void save(const std::string& filename, const std::string& varname, const Eigen::MatrixXf& mat) {
  std::ofstream out(filename.c_str());
  assert(out.is_open());

  /** outputting the header information. **/
  out << "# name: " << varname << std::endl;
  out << "# type: matrix" << std::endl;
  out << "# rows: " << mat.rows() << std::endl;
  out << "# columns: " << mat.cols() << std::endl;

  for (uint32_t r = 0; r < mat.rows(); ++r) {
    for (uint32_t c = 0; c < mat.cols(); ++c) {
      out << " " << mat(r, c);
    }
    out << std::endl;
  }

  out.close();
}

bool load(const std::string& filename, Eigen::MatrixXf& M) {
  std::ifstream in(filename.c_str());
  if (!in.is_open()) {
    std::cerr << "unable to open '" << filename << "'" << std::endl;
    return false;
  }

  std::string line;
  std::vector<std::string> tokens;

  bool rows_read = false, cols_read = false;
  uint32_t rows = 0, cols = 0;

  in.peek();
  while ((!rows_read || !cols_read) && !in.eof()) {
    std::getline(in, line);
    if (line[0] != '#') break;
    tokens = split(line, ":");
    // trim = remove "# " in the begining.
    std::string name = trim(tokens[0], " #");
    std::string value = trim(tokens[1]);

    if (name == "type") {
      if (value != "matrix") {
        std::cerr << "unknown type '" << tokens[1] << "'!" << std::endl;
        return false;
      }
    }

    if (name == "rows") {
      rows_read = true;
      // trim = remove leading spaces
      rows = boost::lexical_cast<uint32_t>(value);
    }

    if (name == "columns") {
      cols_read = true;
      // trim = remove leading spaces
      cols = boost::lexical_cast<uint32_t>(value);
    }
    in.peek();
  }

  if (!rows_read) {
    std::cerr << "missing 'rows' entry!" << std::endl;
    return false;
  }
  if (!cols_read) {
    std::cerr << "missing 'cols' entry!" << std::endl;
    return false;
  }

  M = Eigen::MatrixXf(rows, cols);

  in.peek();
  for (uint32_t r = 0; r < rows; ++r) {
    if (in.eof()) return false;

    std::getline(in, line);
    // trim = remove leading space.
    tokens = split(trim(line));
    if (tokens.size() < cols) return false;

    for (uint32_t c = 0; c < cols; ++c) M(r, c) = boost::lexical_cast<float>(tokens[c]);

    in.peek();
  }

  in.close();

  return true;
}

void save(const std::string& filename, const std::string& varname, const Eigen::MatrixXd& mat) {
  std::ofstream out(filename.c_str());
  assert(out.is_open());

  /** outputting the header information. **/
  out << "# name: " << varname << std::endl;
  out << "# type: matrix" << std::endl;
  out << "# rows: " << mat.rows() << std::endl;
  out << "# columns: " << mat.cols() << std::endl;

  for (uint32_t r = 0; r < mat.rows(); ++r) {
    for (uint32_t c = 0; c < mat.cols(); ++c) {
      out << " " << mat(r, c);
    }
    out << std::endl;
  }

  out.close();
}

bool load(const std::string& filename, Eigen::MatrixXd& M) {
  std::ifstream in(filename.c_str());
  if (!in.is_open()) {
    std::cerr << "unable to open '" << filename << "'" << std::endl;
    return false;
  }

  std::string line;
  std::vector<std::string> tokens;

  bool rows_read = false, cols_read = false;
  uint32_t rows = 0, cols = 0;

  in.peek();
  while ((!rows_read || !cols_read) && !in.eof()) {
    std::getline(in, line);
    if (line[0] != '#') break;
    tokens = split(line, ":");
    // trim = remove "# " in the begining.
    std::string name = trim(tokens[0], " #");
    std::string value = trim(tokens[1]);

    if (name == "type") {
      if (value != "matrix") {
        std::cerr << "unknown type '" << tokens[1] << "'!" << std::endl;
        return false;
      }
    }

    if (name == "rows") {
      rows_read = true;
      // trim = remove leading spaces
      rows = boost::lexical_cast<uint32_t>(value);
    }

    if (name == "columns") {
      cols_read = true;
      // trim = remove leading spaces
      cols = boost::lexical_cast<uint32_t>(value);
    }
    in.peek();
  }

  if (!rows_read) {
    std::cerr << "missing 'rows' entry!" << std::endl;
    return false;
  }
  if (!cols_read) {
    std::cerr << "missing 'cols' entry!" << std::endl;
    return false;
  }

  M = Eigen::MatrixXd(rows, cols);

  in.peek();
  for (uint32_t r = 0; r < rows; ++r) {
    if (in.eof()) return false;

    std::getline(in, line);
    // trim = remove leading space.
    tokens = split(trim(line));
    if (tokens.size() < cols) return false;

    for (uint32_t c = 0; c < cols; ++c) M(r, c) = boost::lexical_cast<double>(tokens[c]);

    in.peek();
  }

  in.close();

  return true;
}

double normal_pdf(double x, double mu, double sigma) {
  static const double sqrt2pi = std::sqrt(2.0 * PI);

  double value = 1. / (sigma * sqrt2pi);
  value *= std::exp(-0.5 * (x - mu) * (x - mu) / (sigma * sigma));

  return value;
}

double exponential_pdf(double x, double lambda) { return lambda * exp(-lambda * x); }

double sigmoid2(double x, double loc, double scale) { return 1.0 / (1.0 + std::exp(-scale * x + scale * loc)); }

Eigen::Matrix2f diag(float val0, float val1) {
  Eigen::Matrix2f m = Eigen::Matrix2f::Zero();

  m(0, 0) = val0;
  m(1, 1) = val1;

  return m;
}

Eigen::Matrix3f diag(float val0, float val1, float val2) {
  Eigen::Matrix3f m = Eigen::Matrix3f::Zero();

  m(0, 0) = val0;
  m(1, 1) = val1;
  m(2, 2) = val2;

  return m;
}
}
}
