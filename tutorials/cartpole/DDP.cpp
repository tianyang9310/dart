/*************************************************************************
    > File Name: DDP.cpp
    > Author: Yang Tian
    > Mail: tianyang9310@gmail.com 
    > Created Time: Mon 23 May 2016 11:49:32 AM EDT
 ************************************************************************/

#include "DDP.h"

DDP::DDP(int T, double m_c, double m_p, double l, double g, double delta_t, WorldPtr mDDPWorld):
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
	delta_t(delta_t),
	mDDPWorld(mDDPWorld)
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
	mu			= 1;
	alpha		= 1;
	coef_upward = 100;
	coef_ctrl   = 1;
	h			= 1e-6;
	
// initial trajectory
// using DARTdynamics
	{
		// reset x to zero
		// When clone the world, x is automatically reset to 0
		WorldPtr DARTdynamicsWorld = mDDPWorld->clone();
		for (int i=0; i<T-1; i++)
		{
			DARTdynamicsWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setForce(u.col(i)[0]);	
			DARTdynamicsWorld->step();

			x(0,i+1) = DARTdynamicsWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->getPosition();
			x(1,i+1) = DARTdynamicsWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->getPosition();
			x(2,i+1) = DARTdynamicsWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->getVelocity();
			x(3,i+1) = DARTdynamicsWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->getVelocity();
		}
	}

// testing derivative
//	std::cout<<"fx: "<<std::endl<<fx<<std::endl;
//	std::cout<<"fu: "<<std::endl<<fu<<std::endl;
//	std::cout<<"Cx: "<<std::endl<<Cx<<std::endl;
//	std::cout<<"Cu: "<<std::endl<<Cu<<std::endl;
//	std::cout<<"Cxx:"<<std::endl<<Cxx<<std::endl;
//	std::cout<<"Cuu:"<<std::endl<<Cuu<<std::endl;
//	std::cout<<"Cux:"<<std::endl<<Cux<<std::endl;
//	backwardpass();
//	std::cout<<"fx: "<<std::endl<<fx<<std::endl;
//	std::cout<<"fu: "<<std::endl<<fu<<std::endl;
//	std::cout<<"Cx: "<<std::endl<<Cx<<std::endl;
//	std::cout<<"Cu: "<<std::endl<<Cu<<std::endl;
//	std::cout<<"Cxx:"<<std::endl<<Cxx<<std::endl;
//	std::cout<<"Cuu:"<<std::endl<<Cuu<<std::endl;
//	std::cout<<"Cux:"<<std::endl<<Cux<<std::endl;
//	std::cin.get();

//	using dynamics
//	for (int i=0;i<T-1;i++)
//	{
//		x.col(i+1)=dynamics(x.col(i),u.col(i));
//	}

// DDP initial data and class variable output
//	std::cout<<"Initial control sequence is"<<std::endl<<u<<std::endl;
//	std::cout<<"Initial state   sequence is"<<std::endl<<x.transpose()<<std::endl;
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
	for (int i=T-1;i>=0;i--)
	{
		derivative(x.col(i),u.col(i),i);
	}

}

void DDP::forwardpass()
{

}

void DDP::DARTdynamics()
{
// No implementation since DART step funtion can directly be called	
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

void DDP::derivative(Eigen::MatrixXd x_i, Eigen::MatrixXd u_i, int mIterator=0)
{
// compute fx, fu, Cx, Cu, Cxx, Cuu, Cux according to x_i and u_i
// Cx, Cu, Cxx, Cuu, Cux are computed according to analytic solution
	Cx.setZero();
	Cu.setZero();
	Cxx.setZero();
	Cuu.setZero();
	Cux.setZero();
	if(!std::isnan(u_i(0)))
	{
		Cu(0)	= coef_ctrl*2*u_i(0);
		Cuu(0)  = coef_ctrl*2;
	}
	Cx(0)	 = 2*x(0);
	Cx(1)  	 = coef_upward*2*(1+std::cos(x(1)))*(-std::sin(x(1)));
	Cx(2)  	 = 2*x(2);
	Cx(3)  	 = 2*x(3);
	Cxx(0,0) = 2;
	Cxx(1,1) = coef_upward*2*(-std::cos(x(1))-std::cos(2*x(1)));
	Cxx(2,2) = 2;
	Cxx(3,3) = 2;
	
// fx, fu are computed according to finite difference
	fx.setZero();
	fu.setZero();
	if(!std::isnan(u_i(0)))
	{
		Eigen::Vector4d f_xt_ut  = x.col(mIterator+1);
		Eigen::Vector4d f_xt_ut_delta;
		Eigen::Matrix4d errorMatrix = h*Eigen::Matrix4d::Identity();
		for (int j=0; j<x_dim; j++)
		{
			Eigen::Vector4d xt_deltaX = x_i + errorMatrix.col(j);
			WorldPtr DARTderivativeWorld = mDDPWorld->clone();
			// restore x_i and u_i
			DARTderivativeWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setPosition(xt_deltaX(0));
			DARTderivativeWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->setPosition(xt_deltaX(1));
			DARTderivativeWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setVelocity(xt_deltaX(2));
			DARTderivativeWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->setVelocity(xt_deltaX(3));

			DARTderivativeWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setForce(u_i(0));	
			DARTderivativeWorld->step();

			f_xt_ut_delta(0) = DARTderivativeWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->getPosition();
			f_xt_ut_delta(1) = DARTderivativeWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->getPosition();
			f_xt_ut_delta(2) = DARTderivativeWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->getVelocity();
			f_xt_ut_delta(3) = DARTderivativeWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->getVelocity();
			fx.col(j) = (f_xt_ut_delta - f_xt_ut)/h;
		}
		{
			double ut_deltaU = u_i(0) + h;
			WorldPtr DARTderivativeWorld = mDDPWorld->clone();
			// restore x_i and u_i
			DARTderivativeWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setPosition(x_i(0));
			DARTderivativeWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->setPosition(x_i(1));
			DARTderivativeWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setVelocity(x_i(2));
			DARTderivativeWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->setVelocity(x_i(3));

			DARTderivativeWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setForce(ut_deltaU);	
			DARTderivativeWorld->step();

			f_xt_ut_delta(0) = DARTderivativeWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->getPosition();
			f_xt_ut_delta(1) = DARTderivativeWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->getPosition();
			f_xt_ut_delta(2) = DARTderivativeWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->getVelocity();
			f_xt_ut_delta(3) = DARTderivativeWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->getVelocity();
			fu.col(0) = (f_xt_ut_delta - f_xt_ut)/h;
		}
	}
	
}

double DDP::cost(Eigen::MatrixXd x_i, Eigen::MatrixXd u_i)
{
	double result;
	if(!std::isnan(u_i(0)))
	{
	result    = std::pow(x_i(0),2) + 
	coef_upward*std::pow((1+std::cos(x_i(1))),2)+ 
	    	    std::pow(x_i(2),2) + 
			    std::pow(x_i(3),2) + 
			(!std::isnan(u_i(0)))*coef_ctrl*std::pow(u_i(0,0),2);
	}
	else
	{
	result    = std::pow(x_i(0),2) + 
	coef_upward*std::pow((1+std::cos(x_i(1))),2)+ 
	    	    std::pow(x_i(2),2) + 
			    std::pow(x_i(3),2);
	}
	return result;
}
