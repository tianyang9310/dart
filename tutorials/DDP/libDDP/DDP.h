#ifndef DDP_H
#define DDP_H 

#include <vector>
#include <cmath>
#include <string>
#include <fstream>
#include <functional>
#include "dart/dart.h"
#include "type.h"

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;
using namespace dart::common;
using namespace dart::math;
using namespace dart::gui;
using namespace std;
using namespace Eigen;


namespace DDP_NSpace
{

class DDP
{
public:
    void trajopt();
    bool backwardpass();
    void forwardpass();
// --------------------------------------------------
    DDP(int T, function<VectorXd(const VectorXd, const VectorXd)> StepDynamics, function<Scalar(const VectorXd, const VectorXd)> StepCost, function<Scalar(const VectorXd)> FinalCost, vector<tuple<MatrixXd, MatrixXd, MatrixXd>> LQR, tuple<VectorXd, VectorXd, int> StateBundle);
    void Derivative(const VectorXd & _xi, const VectorXd & _ui);

    void TrajGenerator();


    function<VectorXd(const VectorXd, const VectorXd)> StepDynamics;
    function<Scalar(const VectorXd, const VectorXd)> StepCost;
    function<Scalar(const VectorXd)> FinalCost;
// --------------------------------------------------
    int T;
    int x_dim;
    int u_dim;
    MatrixXd x;
    MatrixXd u;
    VectorXd C;
    MatrixXd x_new;
    MatrixXd u_new;
    VectorXd C_new;
    Vector2d dV;
    vector<MatrixXd> Vx;
    vector<MatrixXd> Vxx;
    vector<MatrixXd> k;
    vector<MatrixXd> K;
    vector<MatrixXd> Quu_inv;
    vector<function<VectorXd(VectorXd)>> gx;
    MatrixXd fx;
    MatrixXd fu;
    MatrixXd Cx;
    MatrixXd Cu;
    MatrixXd Cxx;
    MatrixXd Cuu;
    MatrixXd Cux;
// --------------------------------------------------
    void setMu();
    double mu;
    double mu_default;
// --------------------------------------------------
    double alpha;
// --------------------------------------------------
    MatrixXd Qf;
    MatrixXd Q;
    MatrixXd R;
    MatrixXd xd;
    MatrixXd x0;
// --------------------------------------------------
    bool isLQR;
};


template<typename FuncType>
MatrixXd FiniteDiff(FuncType Func, VectorXd Var)
{
    // Note:
    //      1. The order of mixed partial derivatives of Hessian
    //          Determine whether it's hessian via the dimensions of output
    //      2. This finite difference function can return Jacobian
    //          In other words, if the output is 1 by 1 and input is x by 1, 
    //          then it returns 1 by x. If the output is x by 1 and input is x by 1,
    //          then it returns x by x
    //      3. Therefore for the output of single value function, extra effort is needed
    //          to make sure the derivative is x by 1. (The returned one is 1 by x)
    //      4. No need to be a member function
    double h = 1e-6;
    int VarDim = Var.rows();
    int OutputDim;
    bool Hessian;
    MatrixXd J_FD;
    if ((Func(Var)).cols() == 1)
    {
        OutputDim = (Func(Var)).rows();
        Hessian   = false;
        J_FD.resize(OutputDim,VarDim);
    }
    else
    {
        OutputDim = (Func(Var)).cols();
        Hessian   = true;
        J_FD.resize(VarDim,OutputDim);
    }

    MatrixXd IndicatorVec(Var.rows(),Var.rows());
    IndicatorVec.setIdentity();

    for (int i=0; i<VarDim; i++)
    {
        if (!Hessian)
        {
            J_FD.col(i) =  (Func(Var+h*IndicatorVec.col(i)) - Func(Var-h*IndicatorVec.col(i)))/(2*h);
        }
        else
        {
            J_FD.row(i) =  (Func(Var+h*IndicatorVec.col(i)) - Func(Var-h*IndicatorVec.col(i)))/(2*h);
        }
    }
    return J_FD.eval();
}


// --------------------------------------------------
template<typename dataFormat_std>
void write2file_std(dataFormat_std data, const string name)
{
    string name_ext = name;
    name_ext.append(".out");
    ofstream outFile(name_ext, ios::out);
    if (outFile.fail())
    {
        dtmsg << "Cannot open "<<name<<" file, please check..."<<endl;
    }
    outFile.precision(8);
    for (size_t i=0; i<data.size(); i++)
    {
        outFile<<data[i]<<endl;
    }
    outFile.close();
}

template<typename dataFormat_eigen>
void write2file_eigen(dataFormat_eigen data, const string name)
{
    string name_ext = name;
    name_ext.append(".out");
    ofstream outFile(name_ext, ios::out);
    if (outFile.fail())
    {
        dtmsg << "Cannot open "<<name<<" file, please check..."<<endl;
    }
    outFile.precision(8);
    outFile<<data<<endl;
    outFile.close();
}

}


#endif
