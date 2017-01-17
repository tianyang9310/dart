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
  double A[] = {-1,0,0,-1,-1,-3,2,5,3,4};
  double b[] = {0,0,-15,100,80};
  int dim_cnst = 5; // num of constraint
  int dim_var = 2; // num of var
  
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

