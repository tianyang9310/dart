/*************************************************************************
    > File Name: DDP.cpp
    > Author: Yang Tian
    > Mail: tianyang9310@gmail.com 
    > Created Time: Mon 23 May 2016 11:49:32 AM EDT
 ************************************************************************/

#include "DDP.h"

DDP::DDP(int T, double m_c, double m_p, double l, double g, double delta_t):
	//x_dim and u_dim are initialized in the header file
	T(T),
	Vx(T),
	Vxx(T),
	k(T),
	K(T),
	m_c(m_c),
	m_p(m_p),
	l(l),
	g(g),
	delta_t(delta_t)
{

	x		= Eigen::MatrixXd::Zero(x_dim,T);
//	u 		= Eigen::MatrixXd::Constant(u_dim,T,1);
	u 		= Eigen::MatrixXd::Random(u_dim,T)*2;
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
	mu		= 1;
	alpha	= 1;
	
// initial trajectory
	for (int i=0;i<T-1;i++)
	{
		x.col(i+1)=dynamics(x.col(i),u.col(i));
	}

// DDP initial data and class variable output
	std::cout<<"Initial control sequence is"<<std::endl<<u<<std::endl;
	std::cout<<"Initial state   sequence is"<<std::endl<<x.transpose()<<std::endl;
//	std::cout<<m_c<<" "<<m_p<<" "<<l<<" "<<g<<" "<<delta_t<<std::endl;
//	std::cout<<C.sum()<<std::endl;
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

Eigen::VectorXd DDP::DARTdynamics(Eigen::MatrixXd x_i, Eigen::MatrixXd u_i)
{

}

Eigen::VectorXd DDP::dynamics(Eigen::MatrixXd x_i, Eigen::MatrixXd u_i)
{
// According to x0 and u(0:T-1) to update x(1:T-1) and get a full trajectory
	Eigen::VectorXd x_i_1(x_i.size());

	double mSin_x_i_1	  = std::sin(x_i(1));
	double mCos_x_i_1 	  = std::cos(x_i(1));
	double mX_i_3_squared = std::pow(x_i(3),2);
	double denomiator     = m_c+m_p*std::pow(mSin_x_i_1,2);
	
	x_i_1(0) = x_i(0) + delta_t * x_i(2);
	x_i_1(1) = x_i(1) + delta_t * x_i(3);

	x_i_1(2) = x_i(2) + delta_t * (m_p*mSin_x_i_1*(l*mX_i_3_squared+g*mCos_x_i_1))/denomiator + delta_t * u_i(0)/denomiator;

	x_i_1(3) = x_i(3) + delta_t * ( - m_p*l*mX_i_3_squared*mSin_x_i_1*mCos_x_i_1 - (m_c+m_p)*g*mCos_x_i_1)/(l*denomiator) + delta_t * ( - mCos_x_i_1 * u_i(0))/(l*denomiator);

	return x_i_1;
}

void DDP::derivative()
{

}

double DDP::cost(Eigen::MatrixXd x_i, Eigen::MatrixXd u_i)
{
	double result;
	if(!std::isnan(u_i(0,0)))
	{
	result    = std::pow(x_i(0),2) + 
			100*std::pow((1+std::cos(x_i(1))),2)+ 
	    	    std::pow(x_i(2),2) + 
			    std::pow(x_i(3),2) + 
			(!std::isnan(u_i(0)))*1*std::pow(u_i(0,0),2);
	}
	else
	{
	result    = std::pow(x_i(0),2) + 
			100*std::pow((1+std::cos(x_i(1))),2)+ 
	    	    std::pow(x_i(2),2) + 
			    std::pow(x_i(3),2);
	}
	return result;
}
