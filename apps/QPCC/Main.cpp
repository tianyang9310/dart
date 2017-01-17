/*
 * Copyright (c) 2011-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>,
 *            Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <vector>

#include "dart/dart.h"
#include "MyWindow.h"
#include "QPCCProblem.h"
#include "SnoptSolver.h"
#include "Var.h"
//#include "apps/QPCC/QPCCSolver.h"

using namespace qpcc;

int main(int argc, char* argv[]) {
//   double A[] = {0.5, 2.0, -1.5, 0.4, -0.6, 1.3, -0.3, 1.7, -2.2};
//   double b[] = {2.1, -0.3, -1.8};
  // double A[] = {1,0,0,0,1,0,0,0,1}; // column major
  // double b[] = {1,2,4};
  
  // double A[] = {1, 1, -1, 2, 2, 1}; // row major
  // double b[] = {2,2,3};
  // double A[] = {1,1,1,1,1,1,1,1};
  // double b[] = {-2};
  // double A[] = {-1,0,0,-1,-1,-3,2,5,3,4};
  // double b[] = {0,0,-15,100,80};
  // double A[] = {1};
  // double b[] = {0.99};
  // double A[] = {3.12, 0.1877, 0.1877, 3.254};
  // double b[] = { 0.00662, 0.006711};
  // double A[] = {
  //              3.999,0.9985, 1.001,    -2,
  //           0.9985, 3.998,    -2,0.9995,
  //            1.001,    -2, 4.002, 1.001,
  //               -2,0.9995, 1.001, 4.001 };
  // double b[] = { 0.01008, 0.009494, 0.07234, 0.07177};
  // double  A[] = {
  //             3.1360,   -2.0370,   0.9723,   0.1096,  -2.0370,   0.9723,
  //          -2.0370,    3.7820,   0.8302,  -0.0257,   2.4730,   0.0105,
  //           0.9723,    0.8302,   5.1250,  -2.2390,  -1.9120,   3.4080,
  //           0.1096,   -0.0257,  -2.2390,   3.1010,  -0.0257,  -2.2390,
  //          -2.0370,    2.4730,  -1.9120,  -0.0257,   5.4870,  -0.0242,
  //           0.9723,    0.0105,   3.4080,  -2.2390,  -0.0242,   3.3860 };
  // double b[] = { 0.1649, -0.0025, -0.0904, -0.0093, -0.0000, -0.0889 };
  double A[] = {
                 4.03, -1.014, -1.898,   1.03, -1.014, -1.898,      1, -1.014, -1.898,     -2, -1.014, -1.898,
             -1.014,  4.885, -1.259,  1.888,   3.81,  2.345, -1.879,  1.281, -2.334,  1.022,  0.206,   1.27,
             -1.898, -1.259,    3.2, -1.032,-0.6849,  1.275,  1.003, 0.6657,  3.774,  1.869,   1.24,   1.85,
               1.03,  1.888, -1.032,   4.03,  1.888, -1.032,     -2,  1.888, -1.032,      1,  1.888, -1.032,
             -1.014,   3.81,-0.6849,  1.888,  3.225,  1.275, -1.879,   1.85,  -1.27,  1.022,  1.265, 0.6907,
             -1.898,  2.345,  1.275, -1.032,  1.275,   4.86,  1.003,  -1.24, 0.2059,  1.869, -2.309,  3.791,
                  1, -1.879,  1.003,     -2, -1.879,  1.003,   3.97, -1.879,  1.003, 0.9703, -1.879,  1.003,
             -1.014,  1.281, 0.6657,  1.888,   1.85,  -1.24, -1.879,  3.187,  1.234,  1.022,  3.755,-0.6714,
             -1.898, -2.334,  3.774, -1.032,  -1.27, 0.2059,  1.003,  1.234,  4.839,  1.869,  2.299,   1.27,
                 -2,  1.022,  1.869,      1,  1.022,  1.869, 0.9703,  1.022,  1.869,   3.97,  1.022,  1.869,
             -1.014,  0.206,   1.24,  1.888,  1.265, -2.309, -1.879,  3.755,  2.299,  1.022,  4.814,  -1.25,
             -1.898,   1.27,   1.85, -1.032, 0.6907,  3.791,  1.003,-0.6714,   1.27,  1.869,  -1.25,  3.212};
  double b[] = { -0.00981, -1.458e-10, 5.357e-10, -0.0098, -1.44e-10, 5.298e-10, -0.009807, -1.399e-10, 5.375e-10, -0.009807, -1.381e-10, 5.316e-10 };

  int dim_cnst = 12; // num of constraint
  int dim_var = 12; // num of var

  // negate b
  for (size_t i = 0; i < dim_var; i++) {
    b[i] = -b[i];
  }
  
  QPCCProblem problem(dim_var, dim_cnst, A, b);
  SnoptSolver solver(&problem);
  solver.solve();

  double x[dim_var];
  for (size_t i = 0; i < dim_var; i++) {
    x[i] = problem.vars()[i]->mVal;
    std::cout << x[i] << std::endl;
  }

  return 0;
}

