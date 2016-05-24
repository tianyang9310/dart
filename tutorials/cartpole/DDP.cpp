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
	u 		= Eigen::MatrixXd::Constant(u_dim,T,1);
	u.col(T-1) = Eigen::VectorXd::Constant(u_dim,std::nan("0"));
	C		= Eigen::VectorXd::Zero(T);
	for (int i=0;i<T;i++)
	{
		Vx[i].setZero();
		Vxx[i].setZero();
		k[i].setZero();
		K[i].setZero();
		C[i]=cost(x.col(i),u.col(i));
	}
	std::cout<<C.sum()<<std::endl;
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
// According to x0 and u(0:T-1) to update x(1:T-1) and get a full trajectory

}

void DDP::derivative()
{

}

double DDP::cost(Eigen::MatrixXd x_i, Eigen::MatrixXd u_i)
{
	double result;
	if(!std::isnan(u_i(0,0)))
	{
	result    = std::pow(x_i(0,0),2) + 
			100*std::pow((1+std::cos(x_i(1,0))),2)	+ 
	    	    std::pow(x_i(2,0),2) + 
			    std::pow(x_i(3,0),2) + 
			(!std::isnan(u_i(0,0)))*1*std::pow(u_i(0,0),2);
	}
	else
	{
	result    = std::pow(x_i(0,0),2) + 
			100*std::pow((1+std::cos(x_i(1,0))),2)	+ 
	    	    std::pow(x_i(2,0),2) + 
			    std::pow(x_i(3,0),2);
	}
	return result;
}
