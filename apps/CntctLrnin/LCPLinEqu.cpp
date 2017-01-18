#include "LCPLinEqu.h"

vector<size_t> sort_indexes(Eigen::VectorXd& v) {
  // initialize original index locations
  vector<size_t> idx(v.size());
  iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  sort(idx.begin(), idx.end(),
       [&v](size_t i1, size_t i2) { return v[i1] > v[i2]; });

  return idx;
}

bool LCPLinEqu(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
               Eigen::VectorXd& z) {
  int dim_var = b.size();
  Eigen::VectorXd z_pattern(dim_var);
  z_pattern.setZero();
  for (size_t i = 0; i < dim_var; i++) {
    if (std::abs(z(i)) < MY_DART_ZERO) {
      z_pattern(i) = 0;
    } else {
      z_pattern(i) = 1;
    }
  }

  int numNonZero = z_pattern.sum();

  // sort z_pattern
  Eigen::VectorXd z_sort = z_pattern;
  vector<size_t> idx = sort_indexes(z_sort);

  /*
   * std::cout << "Before sort: " << z_pattern.transpose() << std::endl;
   * std::cout << "[Wrong] After sort: " << z_sort.transpose() << std::endl;
   * std::cout << "sort index: ";
   * for (size_t i = 0; i < dim_var; i++ ){
   *   std::cout << idx[i] << " ";
   * }
   * std::cout << std::endl;
   */

  // transform
  Eigen::MatrixXd T(dim_var, dim_var);
  T.setIdentity();
  Eigen::MatrixXd T_cache = T;
  for (size_t i = 0; i < dim_var; i++) {
    T.row(i) = T_cache.row(idx[i]);
  }

  Eigen::MatrixXd A_new = T * A * T.inverse();
  Eigen::VectorXd b_new = T * b;

  Eigen::VectorXd ret(dim_var);
  ret.setZero();
  ret.head(numNonZero) = A_new.topLeftCorner(numNonZero, numNonZero)
                             .colPivHouseholderQr()
                             .solve(-b_new.head(numNonZero));

  // restore 
  z = T.colPivHouseholderQr().solve(ret);
  // w = A*z + b;
  
  // Validate z. If fail, return z=0 and false
  bool Validation =
      dart::lcpsolver::YT::validate(A, z, b);
  return Validation;
}
