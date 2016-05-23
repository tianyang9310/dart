/*************************************************************************
    > File Name: DDP.cpp
    > Author: Yang Tian
    > Mail: tianyang9310@gmail.com 
    > Created Time: Mon 23 May 2016 11:49:32 AM EDT
 ************************************************************************/

#include "DDP.h"

DDP::DDP(int T):
	//x_dim and u_dim are initialized in the header file
	T(T),
	Vx(T),
	Vxx(T),
	k(T),
	K(T)
{
	x		= Eigen::MatrixXd::Zero(x_dim,T);
	u 		= Eigen::MatrixXd::Random(u_dim,T);
	u.col(T-1) = Eigen::VectorXd::Constant(u_dim,std::nan("0"));
	for (int i=0;i<T;i++)
	{
		Vx[i].setZero();
		Vxx[i].setZero();
		k[i].setZero();
		K[i].setZero();
	}
	mu		= 1;
	alpha	= 1;
}

void DDP::trajopt()
{
// one iteration of DDP
//------------------------------------------------------------------------------------------------
// backward pass

//------------------------------------------------------------------------------------------------
// forward  pass

}

void DDP::backwardpass()
{

}

void DDP::forwardpass()
{

}

void DDP::dynamics()
{

}

void DDP::derivative()
{

}

void DDP::cost()
{

}
