#ifndef LCPLS_H
#define LCPLS_H

#include "dart/dart.h"
#include "SnoptLPproblem.h"

#define LCPLS_ZERO 1e-12

class LCPLS
{
public:	
	LCPLS(const Eigen::MatrixXd& _A, const Eigen::VectorXd& _b, const std::vector<int> _value);
	virtual ~LCPLS(){};
	void solve();
	Eigen::VectorXd getSolution();


protected:
	int dim_cnst;
	int dim_var;
	Eigen::MatrixXd A;
	Eigen::VectorXd b;
	Eigen::VectorXd z;
	std::vector<int> value;
	std::shared_ptr<SnoptSolver> solver;
};

Eigen::VectorXd value2ub_index(const std::vector<int> value);

#endif