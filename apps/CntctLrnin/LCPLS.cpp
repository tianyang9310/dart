#include "LCPLS.h"

LCPLS::LCPLS(const Eigen::MatrixXd& _A, const Eigen::VectorXd& _b, const std::vector<int> _value):A(_A),b(_b),value(_value)
{
	dim_cnst = A.rows();
	dim_var = A.cols();
    assert(dim_var == dim_cnst);
    z.resize(dim_var);
    z.setZero();
}

void LCPLS::solve() {
    if (b.minCoeff() >= -LCPLS_ZERO) {
        z.setZero();
        return;
    }

    std::vector<int> bas;
    std::vector<int> nonbas;
    bas.clear();
    nonbas.clear();

    Eigen::VectorXd z0 = value2ub_index(value);

    for (int idx=0; idx<dim_var; dim_var++) {
        if (z0(idx) > 0) {
            bas.push_back(idx);
        } else {
            nonbas.push_back(idx);
        }
    }

    Eigen::MatrixXd B = - Eigen::MatrixXd::Identity(dim_var, dim_var);

    Eigen::MatrixXd B_copy = B;
    for (size_t i = 0; i < bas.size(); ++i) {
        B.col(i) = A.col(bas[i]);
    }
    for (size_t i = 0; i < nonbas.size(); ++i) {
        B.col(bas.size() + i) = B_copy.col(nonbas[i]);
    }

    assert(dim_cnst == (bas.size()+nonbas.size()));

    LCPLPproblem problem(dim_var, dim_cnst, B, -b);

    solver = std::make_shared<SnoptSolver>(&problem);
	solver->solve();

    Eigen::VectorXd __z(dim_var);
    __z.setZero();
    for (size_t i = 0; i < dim_var; i++) {
      __z(i) = problem.vars()[i]->mVal;
    }

    if (z.minCoeff() > - LCPLS_ZERO) {
        Eigen::VectorXd _z = Eigen::VectorXd::Zero(2*dim_var);
        for (size_t i=0; i<bas.size(); i++){
            _z(bas[i]) = __z(i);
        }
        z = _z.head(dim_var);
    } else {
        z.setZero();
    }
}

Eigen::VectorXd LCPLS::getSolution() {
    return z;
}

Eigen::VectorXd value2ub_index(const std::vector<int> value){
    int numContactsToLearn = value.size();
    int numBasis = 8;
    Eigen::VectorXd z0 = Eigen::VectorXd::Zero(numContactsToLearn*(numBasis+2));
    std::vector<int> ub_index;
    ub_index.clear();
    for (int i = 0; i<numContactsToLearn; i++) {
        if (value[i] == 9) {
            ub_index.push_back((numBasis+1)*numContactsToLearn+i);
        } else if (value[i] == 8) {
            ub_index.push_back(i);
            for (int j = numContactsToLearn + i * numBasis; j < numContactsToLearn + (i+1)* numBasis; j++) {
                ub_index.push_back(j);
            }            
        } else {
            int offset = numContactsToLearn + i*numBasis;
            ub_index.push_back(i);
            ub_index.push_back(offset+value[i]);
            ub_index.push_back((numBasis+1)*numContactsToLearn+i);
        }
    }

    for (int i = 0; i<ub_index.size(); i++) {
        z0(ub_index[i]) = 1;
    }
    return z0;
}