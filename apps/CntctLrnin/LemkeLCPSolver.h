#ifndef LEMKELCPSOLVER_H
#define LEMKELCPSOLVER_H

#include <cstddef>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include "DepthFirstSearchLCP.h"
#include "MyContactConstraint.h"
#include "MyWindow.h"
#include "SnoptWrapper.h"
#include "dart/dart.h"
#include "utils.h"
#include "../lemkeFix/myLemke.h"

/// Macro controlling prompt and data
#define OUTPUT2FILE
// #define LEMKE_PRINT
// #define ODE_PRINT
// #define IMPULSE_CHANGE

#define RECALL_SOLVE
#define SNOPT_SOLVE
// #define BRUTE_SOLVE

// #define SANITY_CHECK

/*
 * * fn=0, fd=0, lambda=0		contact break
 * * fn=0, fd=0, lambda>0		has relative tangential velocities but no friction
 * * fn=0, fd>0, lambda=0 	X
 * * fn=0, fd>0, lambda>0		X
 * * fn>0, fd=0, lambda=0		static, no relative tangential velocities, no relative tangential acc
 * * fn>0, fd=0, lambda>0		X
 * * fn>0, fd>0, lambda=0		static friction, no relative tangential velocities, relative tangential acc
 * * fn>0, fd>0, lambda>0		slide
 */
//

namespace CntctLrnin {

using namespace dart::constraint;

class MyWindow;

class LemkeLCPSolver : public DantzigLCPSolver {
  public:
  /// Constructor
  LemkeLCPSolver(double _timestep, dart::gui::SimWindow* _mWindow = NULL);

  /// Destructor
  virtual ~LemkeLCPSolver();

  //----------------------------------------------------------------------------
  // Member Function
  //----------------------------------------------------------------------------
  void solve(ConstrainedGroup* _group) override;

  void solveLemke(ConstrainedGroup* _group);
  void solveODE(ConstrainedGroup* _group);


  /// ODE print function
  void print(size_t _n, double* _A, double* _x, double* _lo, double* _hi,
             double* _b, double* w, int* _findex);

  /// Lemke print function
  void print(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
             const Eigen::VectorXd& z, bool Validation, int err);

  /// Permute , negate and augment b
  void permuteNegateAugumentB(double* b, Eigen::VectorXd& lemkeB,
                       const Eigen::VectorXd& preLemkeB, Eigen::MatrixXd& T);

  /// Permute and augment A
  void permuteAugumentA(const Eigen::MatrixXd& preLemkeA,
                    Eigen::MatrixXd& lemkeA, const Eigen::MatrixXd& T,
                    const Eigen::MatrixXd& mu, const Eigen::MatrixXd& E);

  /// scale A matrix for ill-conditioned A
  void scale(Eigen::MatrixXd& A);

  /// Decomposing z
  void decompose(const Eigen::VectorXd& z,
                 std::vector<Eigen::VectorXd>& z_groups);

  /// Lemke fails counter
  int getLemkeFail() { return numLemkeFail; };

  /// Output Lemke solution
  void recordLCPSolve(const Eigen::MatrixXd A, const Eigen::VectorXd z,
                      const Eigen::VectorXd b);

  /// Output files open and close
  void outputFileClose();

  void outputFileOpen();

  /// Return true if the matrix is symmetric
  bool isSymmetric(size_t _n, double* _A);

  /// Return true if the diagonla block of matrix is symmetric
  bool isSymmetric(size_t _n, double* _A, size_t _begin, size_t _end);

  //----------------------------------------------------------------------------
  // Member Variable
  //----------------------------------------------------------------------------
  /// For functionality of Lemke
  int numBasis;

  /// Statistical of funcationality of Lemke
  int numLemkeFail;

  /// Prompt precision
  int mPrecision;

#ifdef OUTPUT2FILE
  /// For output file use
  std::vector<std::shared_ptr<std::fstream>> outputFiles;

  std::vector<int> counters;

  int dataSize;

  int numDesiredFiles;
#endif

  /// For debugging use
  dart::gui::SimWindow* mWindow;
};
}

#endif  // LEMKELCPSOLVER_H
