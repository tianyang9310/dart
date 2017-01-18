#ifndef LCPLINEQU_H
#define LCPLINEQU_H

#include "dart/dart.h"
#include "../Lemke_Fix/MyLemke.h"
#include "utils.h"

using namespace std;

/// return sort index
vector<size_t> sort_indexes(Eigen::VectorXd& v);

/// Here input argument should be the pattern
bool LCPLinEqu(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
               Eigen::VectorXd& z);


#endif
