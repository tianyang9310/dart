#ifndef CSVPARSER
#define CSVPARSER

#include <Eigen/Dense>
#include <fstream>
#include <vector>
#include "dart/dart.h"

using namespace Eigen;

// Loading Eigen Matrix from file
// usage: load_csv<Eigen::MatrixXd>("/tmp/A.csv");
template <typename M>
M load_csv(const std::string& path) {
  std::ifstream indata;
  indata.open(path);
  std::string line;
  std::vector<double> values;
  uint rows = 0;
  while (std::getline(indata, line)) {
    std::stringstream lineStream(line);
    std::string cell;
    while (std::getline(lineStream, cell, ',')) {
      values.push_back(std::stod(cell));
    }
    ++rows;
  }
  return Map<const Matrix<typename M::Scalar, M::RowsAtCompileTime,
                          M::ColsAtCompileTime, RowMajor>>(
      values.data(), rows, values.size() / rows);
}

#endif
