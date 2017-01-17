#ifndef QPCC_QPCCPROBLEM_H_
#define QPCC_QPCCPROBLEM_H_

#include <Eigen/Dense>
#include "Problem.h"
#include "dart/dart.h"

namespace qpcc {

class QPCCProblem : public Problem {
public:
  QPCCProblem(size_t dim_var, size_t dim_cnst, double* A, double* b);
  virtual ~QPCCProblem();
  virtual void update(double* _coefs);
  
protected:

};
}
#endif
