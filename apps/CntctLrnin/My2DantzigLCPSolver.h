#ifndef MY2DANTZIGLCPSOLVER
#define MY2DANTZIGLCPSOLVER

#include <cstddef>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include "My2ContactConstraint.h"
#include "dart/dart.h"
#include "utils.h"

// #include "dart/constraint/LCPSolver.h"
// #include "dart/constraint/DantzigLCPSolver.h"
// #include "dart/constraint/ConstraintBase.h"
// #include "dart/constraint/ConstrainedGroup.h"
//
// #include "dart/config.h"
// #include "dart/common/Console.h"
// #include "dart/lcpsolver/Lemke.h"
#include "../Lemke_Fix/MyLemke.h"
// #include "dart/lcpsolver/lcp.h"

// #include "MyDantzigLCPSolver.h"

using namespace dart::constraint;

class MyWindow;

class My2DantzigLCPSolver : public DantzigLCPSolver {
  public:
  /// Constructor
  My2DantzigLCPSolver(double _timestep);

  /// Destructor
  virtual ~My2DantzigLCPSolver();

  //----------------------------------------------------------------------------
  // Member Function
  //----------------------------------------------------------------------------
  void solve(ConstrainedGroup* _group) override;

  void print(size_t _n, double* _A, double* _x, double* _lo, double* _hi,
             double* _b, double* w, int* _findex);

  /// Return true if the matrix is symmetric
  bool isSymmetric(size_t _n, double* _A);

  /// Return true if the diagonla block of matrix is symmetric
  bool isSymmetric(size_t _n, double* _A, size_t _begin, size_t _end);

  /// Permute , negate and augment b
  void PermuteNegAug_b(double* b, Eigen::VectorXd& Lemke_b,
                       const Eigen::VectorXd& Pre_Lemke_b, Eigen::MatrixXd& T);

  /// Permute and augment A
  void PermuteAug_A(const Eigen::MatrixXd& Pre_Lemke_A,
                    Eigen::MatrixXd& Lemke_A, const Eigen::MatrixXd& T,
                    const Eigen::MatrixXd& mu, const Eigen::MatrixXd& E);

  //----------------------------------------------------------------------------
  // Member Variable
  //----------------------------------------------------------------------------
  int numBasis;
};

#endif
