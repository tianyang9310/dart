#include "QP.h"

QP::QP(const Eigen::MatrixXd &_A, const Eigen::VectorXd &_b)
{
    mDim = _b.rows();
    mProblem = std::make_shared<Problem>(mDim);
    mProblem->setLowerBounds(Eigen::VectorXd::Zero(mDim));
    mProblem->setInitialGuess(9.8*Eigen::VectorXd::Ones(mDim));

    FunctionPtr obj = std::make_shared<ObjFunc>(_A,_b);
    mProblem->setObjective(obj);

    for (int i=0; i<mDim; i++)
    {
        mProblem->addIneqConstraint(std::make_shared<CnstrFunc>((_A.row(i)).transpose(),_b(i)));
    }

    mSolver = std::make_shared<NloptSolver>(mProblem, nlopt::LD_MMA);
    // mSolver = std::make_shared<IpoptSolver>(mProblem);
    mSolver->solve();

    double minObj = mProblem->getOptimumValue();
    mf = mProblem->getOptimalSolution();
    std::cout<<"min Objective is "<<minObj<<std::endl;
    std::cout<<"f is "<<mf<<std::endl;
}
