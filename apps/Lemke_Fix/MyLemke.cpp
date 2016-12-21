/*
 * Copyright (c) 2011-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jie (Jay) Tan <jtan34@cc.gatech.edu>,
 *            Yunfei Bai <byf1658@gmail.com>
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

#include <cmath>
#include <iostream>
#include <vector>

#include "dart/math/Helpers.h"
#include "MyLemke.h"

#ifndef isnan
# define isnan(x) \
  (sizeof (x) == sizeof (long double) ? isnan_ld (x) \
  : sizeof (x) == sizeof (double) ? isnan_d (x) \
  : isnan_f(x))

static inline int isnan_f(float _x) { return _x != _x; }

static inline int isnan_d(double _x) { return _x != _x; }

static inline int isnan_ld(long double _x) { return _x != _x; }

#endif

#ifndef isinf
# define isinf(x) \
  (sizeof (x) == sizeof (long double) ? isinf_ld (x) \
  : sizeof (x) == sizeof (double) ? isinf_d (x) \
  : isinf_f(x))

static inline int isinf_f(float _x) { return !isnan (_x) && isnan (_x - _x); }

static inline int isinf_d(double _x) { return !isnan (_x) && isnan (_x - _x); }

static inline int isinf_ld(long double _x) { return !isnan (_x) && isnan (_x - _x); }

#endif

namespace dart {
namespace lcpsolver {
namespace YT {

// double RandDouble(double _low, double _high) {
//  double temp;

//  /* swap low & high around if the user makes no sense */
//  if (_low > _high) {
//    temp = _low;
//    _low = _high;
//    _high = temp;
//  }

//  /* calculate the random number & return it */
//  temp = (rand() / (static_cast<double>(RAND_MAX) + 1.0))
//         * (_high - _low) + _low;
//  return temp;
// }

int Lemke(const Eigen::MatrixXd &_M, const Eigen::VectorXd &_q,
      Eigen::VectorXd *_z) {
int n = _q.size();

const double zer_tol = 1e-6;
const double piv_tol = 1e-16;
int maxiter = 1000;
int err = 0;

if (_q.minCoeff() >= 0) {
    // LOG(INFO) << "Trivial solution exists.";
    *_z = Eigen::VectorXd::Zero(n);
    return err;
}

// solve trivial case for n=1
//   if (n==1){
//     if (_M(0)>0){
//         *_z = (- _q(0)/_M(0) )*Eigen::VectorXd::Ones(n);
//         return err;
//     } else {
//         *_z = Eigen::VectorXd::Zero(n);
//         err = 4; // no solution
//         return err;
//     }
//   }

*_z = Eigen::VectorXd::Zero(2 * n);
int iter = 0;
// double theta = 0;
double ratio = 0;
int leaving = 0;
Eigen::VectorXd Be = Eigen::VectorXd::Constant(n, 1);
Eigen::VectorXd x = _q;
std::vector<int> bas;
std::vector<int> nonbas;

int t = 2 * n;
int entering = t;

bas.clear();
nonbas.clear();

// TODO: here suppose initial guess z0 is [0,0,0,...], this contradicts to ODE's w always initilized as 0
for (int i = 0; i < n; ++i) {
    nonbas.push_back(i);
}

Eigen::MatrixXd B = -Eigen::MatrixXd::Identity(n, n);

if (!bas.empty()) {
    Eigen::MatrixXd B_copy = B;
    for (size_t i = 0; i < bas.size(); ++i) {
        B.col(i) = _M.col(bas[i]);
    }
    for (size_t i = 0; i < nonbas.size(); ++i) {
        B.col(bas.size() + i) = B_copy.col(nonbas[i]);
    }
    // TODO: check the condition number to return err = 3
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(B);
    double cond = svd.singularValues()(0)
        / svd.singularValues()(svd.singularValues().size() - 1);
    if (cond > 1e16) {
        (*_z) = Eigen::VectorXd::Zero(n);
        err = 3;
        return err;
    }
    x = -B.householderQr().solve(_q);
}

// Check if initial basis provides solution
if (x.minCoeff() >=0 ) {
    Eigen::VectorXd __z = Eigen::VectorXd::Zero(2 * n);
    for (size_t i = 0; i < bas.size(); ++i) {
        (__z).row(bas[i]) = x.row(i);
    }
    (*_z) = __z.head(n);
    return err;
}

// Determine initial leaving variable
Eigen::VectorXd minuxX = -x;
int lvindex;
double tval = minuxX.maxCoeff(&lvindex);
for (size_t i = 0; i < nonbas.size(); ++i) {
    bas.push_back(nonbas[i] + n);
}
leaving = bas[lvindex];

bas[lvindex] = t; // pivoting in the artificial variable

Eigen::VectorXd U = Eigen::VectorXd::Zero(n);
for (int i = 0; i < n; ++i) {
    if (x[i] < 0)
        U[i] = 1;
}
Be = -(B * U);
x += tval * U;
x[lvindex] = tval;
B.col(lvindex) = Be;

for (iter = 0; iter < maxiter; ++iter) {
    if (leaving == t) {
        break;
    } else if (leaving < n) {
        entering = n + leaving;
        Be = Eigen::VectorXd::Zero(n);
        Be[leaving] = -1;
    } else {
        entering = leaving - n;
        Be = _M.col(entering);
    }

    Eigen::VectorXd d = B.householderQr().solve(Be);

    // Find new leaving variable
    std::vector<int> j;
    for (int i = 0; i < n; ++i) {
        if (d[i] > piv_tol)
            j.push_back(i);
    }
    if (j.empty()) // no new pivots - ray termination
    {
        err = 2;
        break;
    }

    size_t jSize = j.size();
    Eigen::VectorXd minRatio(jSize);
    for (size_t i = 0; i < jSize; ++i) {
        minRatio[i] = (x[j[i]] + zer_tol) / d[j[i]];
    }
    double theta = minRatio.minCoeff();

    std::vector<int> tmpJ;
    std::vector<double> tmpd;
    for (size_t i = 0; i < jSize; ++i) {
        if (x[j[i]] / d[j[i]] <= theta) {
            tmpJ.push_back(j[i]);
            tmpd.push_back(d[j[i]]);
        }
    }

//    if (tmpJ.empty())
//    {
//      LOG(WARNING) << "tmpJ should never be empty!!!";
//      LOG(WARNING) << "dumping data:";
//      LOG(WARNING) << "theta:" << theta;
//      for (int i = 0; i < jSize; ++i)
//      {
//        LOG(WARNING) << "x(" << j[i] << "): " << x[j[i]] << "d: " << d[j[i]];
//      }
//    }

    j = tmpJ;
    jSize = j.size();
    if (jSize == 0) {
        err = 4;
        break;
    }
    lvindex = -1;

    // Check if artificial among these
    for (size_t i = 0; i < jSize; ++i) {
        if (bas[j[i]] == t)
            lvindex = i;
    }

    if (lvindex != -1) {
        lvindex = j[lvindex]; // Always use artificial if possible
    } else {
        theta = tmpd[0];
        lvindex = 0;
        for (size_t i = 0; i < jSize; ++i) {
            if (tmpd[i] - theta > piv_tol) {  // Bubble sorting
                theta = tmpd[i];
                lvindex = i;
            }
        }
        lvindex = j[lvindex]; // choose the first if there are multiple
    }

    leaving = bas[lvindex];

    ratio = x[lvindex] / d[lvindex];

// //YT    bool bDiverged = false;
// //YT    for (int i = 0; i < n; ++i) {
// //YT      if (isnan(x[i]) || isinf(x[i])) {
// //YT        bDiverged = true;
// //YT        break;
// //YT      }
// //YT    }
// //YT    if (bDiverged) {
// //YT      err = 4;
// //YT      break;
// //YT    }

    // Perform pivot
    x = x - ratio * d;
    x[lvindex] = ratio;
    B.col(lvindex) = Be;
    bas[lvindex] = entering;
}

if (iter >= maxiter && leaving != t) {
    err = 1;
}

if (err == 0) {
    for (size_t i = 0; i < bas.size(); ++i) {
        if (bas[i] < _z->size()) {
            (*_z)[bas[i]] = x[i];
        }
    }

    Eigen::VectorXd __z = _z->head(n);
    *_z = __z;

    if (!validate(_M, *_z, _q)) {
        // _z = VectorXd::Zero(n);
        err = 3;
    }
} else {
    *_z = Eigen::VectorXd::Zero(n);  // solve failed, return a 0 vector
}

//  if (err == 1)
//    LOG(ERROR) << "LCP Solver: Iterations exceeded limit";
//  else if (err == 2)
//    LOG(ERROR) << "LCP Solver: Unbounded ray";
//  else if (err == 3)
//    LOG(ERROR) << "LCP Solver: Solver converged with numerical issues. "
//               << "Validation failed.";
//  else if (err == 4)
//    LOG(ERROR) << "LCP Solver: Iteration diverged.";

return err;
}

bool validate(const Eigen::MatrixXd &_M, const Eigen::VectorXd &_z,
          const Eigen::VectorXd &_q) {
const double threshold = 1e-6;
int n = _z.size();

Eigen::VectorXd w = _M * _z + _q;
for (int i = 0; i < n; ++i) {
    if (w(i) < -threshold || _z(i) < -threshold)
        return false;
    if (std::abs(w(i) * _z(i)) > threshold)
        return false;
}
return true;
}

bool validate(const Eigen::MatrixXd &_M, const Eigen::VectorXd &_z,
          const Eigen::VectorXd &_q, double & err_dist) {
const double threshold = 1e-6;
int n = _z.size();
err_dist = 0.0;

Eigen::VectorXd w = _M * _z + _q;
for (int i = 0; i < n; ++i) {
    if (w(i) < 0) {
      err_dist += -w(i);
    }
    if (w(i) < -threshold) {
        return false;
    } 
    if (_z(i) < 0) {
      err_dist += -_z(i);
    }
    if (_z(i) < -threshold) {
        return false;
    }
    err_dist += std::abs(w(i)*_z(i));
    if (std::abs(w(i) * _z(i)) > threshold) {
        return false;
    }
}

return true;
}

}  // namespace YT
}  // namespace lcpsolver
}  // namespace dart