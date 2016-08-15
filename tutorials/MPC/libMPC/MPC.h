#ifndef MPC_H
#define MPC_H
#include <iostream>
#include <vector>
#include <functional>
#include "dart/dart.h"
#include "../../DDP/libDDP/DDP.h"
#include "../../DDP/libDDP/type.h"

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;
using namespace dart::common;
using namespace dart::math;
using namespace dart::gui;

using namespace std;
using namespace Eigen;
using namespace DDP_NSpace;
namespace MPC_NSpace
{

class MPC
{
public:
    MPC(int _local_T, int _T, int _x_dim, int _u_dim, Vector4d _x0, Vector4d _xd, MatrixXd _Q, Matrix<double,1,1> _R, MatrixXd _Qf, function<VectorXd(const VectorXd, const VectorXd)> _mStepDynamics, function<Scalar(const VectorXd, const VectorXd)> _mStepCost, function<Scalar(const VectorXd)> _mFinalCost);
    void run();
    void inner_run(int i);
    MatrixXd getX();
    MatrixXd getU();
    

// --------------------------------------------------
    int DDPiters;
    unique_ptr<DDP> mDDP;
    int local_T;
    int T;
    MatrixXd x;
    MatrixXd u;
    Vector4d x0; 
    Vector4d xd; 
    MatrixXd Q; 
    Matrix<double,1,1> R; 
    MatrixXd Qf;
	vector<tuple<MatrixXd, MatrixXd, MatrixXd>> LQR;
    function<VectorXd(const VectorXd, const VectorXd)> mStepDynamics;
    function<Scalar(const VectorXd, const VectorXd)> mStepCost;
    function<Scalar(const VectorXd)> mFinalCost;


};

}
#endif
