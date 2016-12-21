#ifndef MY2DANTZIGLCPSOLVER
#define MY2DANTZIGLCPSOLVER

#include <cstddef>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include "MyContactConstraint.h"
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
// #include "../Lemke_Fix/MyLemke.h"
// #include "dart/lcpsolver/lcp.h"

// #include "MyDantzigLCPSolver.h"

using namespace dart::constraint;

class MyWindow;

class My2DantzigLCPSolver : public DantzigLCPSolver {
  public:
  My2DantzigLCPSolver(double _timestep);
  virtual ~My2DantzigLCPSolver();
  void solve(ConstrainedGroup* _group) override;
  void print(size_t _n, double* _A, double* _x, double* _lo, double* _hi,
             double* _b, double* w, int* _findex,
             std::shared_ptr<std::fstream> ODE_FILE);

  /// Return true if the matrix is symmetric
  bool isSymmetric(size_t _n, double* _A);

  /// Return true if the diagonla block of matrix is symmetric
  bool isSymmetric(size_t _n, double* _A, size_t _begin, size_t _end);
};

#endif
