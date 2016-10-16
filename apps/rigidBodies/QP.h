#ifndef _QP_H_RESTINGCONTACT
#define _QP_H_RESTINGCONTACT

#include <iostream>
#include "dart/dart.h"
#include "dart/lcpsolver/Lemke.h"
#include "dart/optimizer/Function.h"
#include "dart/optimizer/Problem.h"
#include "dart/optimizer/Solver.h"
#include "dart/optimizer/nlopt/NloptSolver.h"
#include "dart/optimizer/ipopt/IpoptSolver.h"

using namespace dart::optimizer;

class ObjFunc : public Function
{
public:
    ObjFunc(const Eigen::MatrixXd &_A, const Eigen::VectorXd &_b):Function(),mA(_A),mb(_b)
    {
        mDim = _b.rows();
    }

    virtual ~ObjFunc(){};
    
    virtual double eval(const Eigen::VectorXd& _x) override
    {
        double res = _x.dot(mA * _x + mb);
        // std::cout<<"In objective"<<std::endl;
        // std::cout<<mA<<std::endl;
        // std::cout<<mb<<std::endl;
        // std::cout<<_x<<std::endl;
        // std::cout<<res<<std::endl;
        // std::cin.get();
        return res;
    }

    virtual void evalGradient(const Eigen::VectorXd& _x, Eigen::Map<Eigen::VectorXd> _grad) override
    {
        Eigen::VectorXd tmpGrad = (2*mA*_x + mb).eval(); 
        for (int i=0; i<mDim; i++)
        {
            _grad[i] = tmpGrad(i);
        }
    }

    Eigen::MatrixXd mA;
    Eigen::VectorXd mb;
    int mDim;
};

class CnstrFunc : public Function
{
public:
    CnstrFunc(const Eigen::VectorXd &_A, const double &_b):Function(),mA(_A),mb(_b)
    {
        mDim = _A.rows();
    }
    
    virtual ~CnstrFunc(){};

    virtual double eval(const Eigen::VectorXd& _x) override
    {
        double res = -(mA.dot(_x) + mb);
        // std::cout<<"In constraint"<<std::endl;
        // std::cout<<mA<<std::endl;
        // std::cout<<mb<<std::endl;
        // std::cout<<_x<<std::endl;
        // std::cout<<res<<std::endl;
        // std::cin.get();
        return res;
    }

    virtual void evalGradient(const Eigen::VectorXd& _x, Eigen::Map<Eigen::VectorXd> _grad) override
    {
        Eigen::VectorXd tmpGrad = -mA;
        for (int i=0; i<mDim; i++)
        {
            _grad[i] = tmpGrad(i);
        }
    }

    Eigen::VectorXd mA;
    double mb;
    int mDim;
};


class QP
{
public:
    QP(const Eigen::MatrixXd &_A, const Eigen::VectorXd &_b);
    virtual ~QP(){};
    
    Eigen::VectorXd mf;
    
    std::shared_ptr<Solver> mSolver;    
    std::shared_ptr<Problem> mProblem;
    int mDim;    
};

#endif // _QP_H_RESTINGCONTACT
