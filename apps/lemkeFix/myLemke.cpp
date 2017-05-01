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
#include <glog/logging.h>
#include "dart/math/Helpers.h"
#include "myLemke.h"

/*
 * #ifndef isnan
 * # define isnan(x) \
 *   (sizeof (x) == sizeof (long double) ? isnan_ld (x) \
 *   : sizeof (x) == sizeof (double) ? isnan_d (x) \
 *   : isnan_f(x))
 * 
 * static inline int isnan_f(float _x) { return _x != _x; }
 * 
 * static inline int isnan_d(double _x) { return _x != _x; }
 * 
 * static inline int isnan_ld(long double _x) { return _x != _x; }
 * 
 * #endif
 * 
 * #ifndef isinf
 * # define isinf(x) \
 *   (sizeof (x) == sizeof (long double) ? isinf_ld (x) \
 *   : sizeof (x) == sizeof (double) ? isinf_d (x) \
 *   : isinf_f(x))
 * 
 * static inline int isinf_f(float _x) { return !isnan (_x) && isnan (_x - _x); }
 * 
 * static inline int isinf_d(double _x) { return !isnan (_x) && isnan (_x - _x); }
 * 
 * static inline int isinf_ld(long double _x) { return !isnan (_x) && isnan (_x - _x); }
 * 
 * #endif
 */

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
      Eigen::VectorXd *_z, Eigen::VectorXd* z0) {
int n = _q.size();

// regularization x/d such that it is comparable for some very small x, magnify 
// the difference of x, otherwise all small x have almost the same x/d. On the
// other hand, we can find a list of small x. In essence it cannot be too large
const double zer_tol = 1e-6; 
// comparing piv_tol to zero, s.t. as small as possible
const double piv_tol = 1e-12; 

int maxiter = std::min(1000, 25*n);
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

if (z0) {
    for (int i = 0; i < n; ++i) {
        if ((*z0)(i) > piv_tol /*0*/) {
            bas.push_back(i);
        } else {
            nonbas.push_back(i);
        }
    }
} else {
// TODO: here suppose initial guess z0 is [0,0,0,...], this contradicts to ODE's w always initilized as 0
    for (int i = 0; i < n; ++i) {
        nonbas.push_back(i);
    }
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
        std::cout << "Fail due to cond # before iterations" << std::endl;
        (*_z) = Eigen::VectorXd::Zero(n);
        err = 3;
        return err;
    }
    // x = -B.householderQr().solve(_q);
    x = -B.colPivHouseholderQr().solve(_q);
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

    // Eigen::VectorXd d = B.householderQr().solve(Be);
    Eigen::VectorXd d = B.colPivHouseholderQr().solve(Be);

    // Find new leaving variable
    std::vector<int> j;
    for (int i = 0; i < n; ++i) {
        if (d[i] > piv_tol)
            j.push_back(i);
    }
    if (j.empty()) // no new pivots - ray termination
    {
        LOG(INFO) << "Fails due to empty j in the iterations, which means second ray termination!" << std::endl;
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
        if (bas[j[i]] == t) {
            lvindex = i;
            break;
        }
    }

    if (lvindex != -1) {
        lvindex = j[lvindex]; // Always use artificial if possible
    } else {
        theta = tmpd[0];
        lvindex = 0;
        for (size_t i = 0; i < jSize; ++i) {
            if (tmpd[i] - theta > piv_tol) {  // Bubble sorting: return the first max
                theta = tmpd[i];
                lvindex = i;
            }
        }

        // random choose for multiple 
        std::vector<int> tmpd_max_idx;
        tmpd_max_idx.clear();
        for (size_t i = 0; i < jSize; i++) {
          if (std::abs(theta - tmpd[i]) < piv_tol) {
            tmpd_max_idx.push_back(i);
          } 
        }
        lvindex = tmpd_max_idx[std::rand() % tmpd_max_idx.size()];

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

// std::cout << std::endl << iter << "th iteration Lemke" << std::endl 
//           << std::endl;

if (iter >= maxiter && leaving != t) {
  LOG(INFO) << "Fails due to exceeding max iterations and still cannot find a solution!" << std::endl;
  err = 1;
}

if (err == 0) {
    for (size_t i = 0; i < bas.size(); ++i) {
      // TODOs: here if statement could be removed
        if (bas[i] < _z->size()) {
            (*_z)[bas[i]] = x[i];
        }
    }

    Eigen::VectorXd __z = _z->head(n);
    *_z = __z;

    if (!validate(_M, _q, *_z)) {
        // _z = VectorXd::Zero(n);
        LOG(INFO) << "Fails due to validation after iterations" << std::endl; 
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

bool validate(const Eigen::MatrixXd &_M, const Eigen::VectorXd &_q,
          const Eigen::VectorXd &_z) {
const double threshold = 1e-6;
int n = _z.size();

if (Eigen::isnan(_z.array()).any()) {
    return false;
}

Eigen::VectorXd w = _M * _z + _q;
for (int i = 0; i < n; ++i) {
    if (w(i) < -threshold || _z(i) < -threshold) {
        return false;
    }
    if (std::abs(w(i) * _z(i)) > threshold) {
        return false;
    }
    if (std::isnan(_z(i))) {
      return false;
    }
}
return true;
}

}  // namespace YT
}  // namespace lcpsolver
}  // namespace dart
