#include "MPC.h"

namespace MPC_NSpace
{
MPC::MPC(int _local_T, int _T, int _x_dim, int _u_dim, Vector4d _x0, Vector4d _xd, MatrixXd _Q, Matrix<double,1,1> _R, MatrixXd _Qf, function<VectorXd(const VectorXd, const VectorXd)> _mStepDynamics, function<Scalar(const VectorXd, const VectorXd)> _mStepCost, function<Scalar(const VectorXd)> _mFinalCost):
    local_T(_local_T),
    T(_T),
    x0(_x0), 
    xd(_xd), 
    Q(_Q), 
    R(_R), 
    Qf(_Qf),
    mStepDynamics(_mStepDynamics),
    mStepCost(_mStepCost),
    mFinalCost(_mFinalCost)
{
    DDPiters = 1;
    x.setZero(_x_dim, T);
    u.setZero(_u_dim, T);
    x.col(0) = x0;

	LQR.push_back(make_tuple(Q,R,Qf));
	mDDP = unique_ptr<DDP>(new DDP(local_T, mStepDynamics, mStepCost, mFinalCost, LQR, make_tuple(x.col(0),xd,1)));
}

void MPC::run()
{
    // after executing run(), x and u could be returned;
    
    dtmsg<<"Using Model Predictive Control to realize Cart Pole Balancing Problem..."<<endl;
    cout<<endl<<endl<<endl;

    for (int i=0; i<T-1; i++)
    {
        inner_run(i);
    }
}

void MPC::inner_run(int i)
{
    cout<<" "<<i<<" step of traj"<<endl;
	mDDP = unique_ptr<DDP>(new DDP(local_T, mStepDynamics, mStepCost, mFinalCost, LQR, make_tuple(x.col(i),xd,1)));

    if (i<T-local_T)
    {
        int u_dim = u.rows();
        mDDP->u = u.block(u_dim-1,i,u_dim,local_T);
        mDDP->u.col(local_T-1) = VectorXd::Constant(u_dim, nan("0"));
        mDDP->TrajGenerator();
    }

    for (int _DDP_iter=0; _DDP_iter<DDPiters; _DDP_iter++)
    {
        mDDP->trajopt();
    }

    u.col(i)=mDDP->u.col(0);
    x.col(i+1)=mStepDynamics(mDDP->x.col(0),mDDP->u.col(0));
}

MatrixXd MPC::getU()
{
    int u_dim = u.rows();
    u.col(T-1) = VectorXd::Constant(u_dim, nan("0"));
    return u;
}

}

