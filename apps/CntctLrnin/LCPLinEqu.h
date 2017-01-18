#ifndef LCPLINEQU_H
#define LCPLINEQU_H

#include "../Lemke_Fix/MyLemke.h"
#include "dart/dart.h"
#include "utils.h"

using namespace std;

/// return sort index
vector<size_t> sort_indexes(Eigen::VectorXd& v);

/// Here input argument should be the pattern
bool LCPLinEqu(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
               Eigen::VectorXd& z);

/// DFS search to find LCP solution
//  z is the pattern, A,b are given constant, ret are solution lists
bool DFS(Eigen::VectorXd& z, size_t index, const Eigen::MatrixXd& A,
         const Eigen::VectorXd& b, vector<Eigen::VectorXd>& ret_list);

#endif
