#include "DDP.h"

DDP::DDP(int T, WorldPtr mDDPWorld):
	T(T),
	Vx(T),
	Vxx(T),
	k(T),
	K(T),
	mDDPWorld(mDDPWorld)
{
	Q.setZero();
	Q(0,0)		= 0.1;
	Q(1,1)		= 0.1;
	Qf.setIdentity()*5;
	Qf(1,1)		= 100;
	R.setIdentity();
	x_f.setZero();
	x_f(1)      = M_PI;
	x_0.setZero();
	mu    = 0;
	alpha = 1;
	h     = 1e-6;
// -----------------------------------------------------------------------------------------------
	x		= Eigen::MatrixXd::Zero(x_dim,T);
	u 		= Eigen::MatrixXd::Constant(u_dim,T,0);
	//u 		= Eigen::MatrixXd::Random(u_dim,T)*150;
	u.col(T-1) = Eigen::VectorXd::Constant(u_dim,std::nan("0"));
	C		= Eigen::VectorXd::Zero(T);
// produce initial trajectory using DARTdynamics
	x		= LQRdynamics(x_0, u);
// -----------------------------------------------------------------------------------------------
	x_new   = Eigen::MatrixXd::Zero(x_dim,T);
	u_new   = Eigen::MatrixXd::Zero(u_dim,T);
	C_new	= Eigen::VectorXd::Zero(T);
// -----------------------------------------------------------------------------------------------
	dV.setZero();
	for (int i=0;i<T;i++)
	{
		Vx[i].setZero();
		Vxx[i].setZero();
		k[i].setZero();
		K[i].setZero();
		if (i<T-1)
		{
			C[i]=LQRcost(x.col(i),u.col(i));
		}
	}
	C[T-1]  = 0.5*(x.col(T-1) - x_f).transpose()*Qf*(x.col(T-1) - x_f);
// -----------------------------------------------------------------------------------------------
// testing derivative
	/*
	std::cout<<"fx: "<<std::endl<<fx<<std::endl;
	std::cout<<"fu: "<<std::endl<<fu<<std::endl;
	std::cout<<"Cx: "<<std::endl<<Cx<<std::endl;
	std::cout<<"Cu: "<<std::endl<<Cu<<std::endl;
	std::cout<<"Cxx:"<<std::endl<<Cxx<<std::endl;
	std::cout<<"Cuu:"<<std::endl<<Cuu<<std::endl;
	std::cout<<"Cux:"<<std::endl<<Cux<<std::endl;
	for (int i=T-1;i>=0;i--)
	{
		derivative(x.col(i),u.col(i),i);
	}
	std::cout<<"fx: "<<std::endl<<fx<<std::endl;
	std::cout<<"fu: "<<std::endl<<fu<<std::endl;
	std::cout<<"Cx: "<<std::endl<<Cx<<std::endl;
	std::cout<<"Cu: "<<std::endl<<Cu<<std::endl;
	std::cout<<"Cxx:"<<std::endl<<Cxx<<std::endl;
	std::cout<<"Cuu:"<<std::endl<<Cuu<<std::endl;
	std::cout<<"Cux:"<<std::endl<<Cux<<std::endl;
	std::cin.get();
	*/

// -----------------------------------------------------------------------------------------------
// DDP initial data and some variable output
//	std::cout<<"Initial control sequence is"<<std::endl<<u<<std::endl;
//	std::cout<<"Initial state   sequence is"<<std::endl<<x.transpose()<<std::endl;
//	std::cout<<m_c<<" "<<m_p<<" "<<l<<" "<<g<<" "<<delta_t<<std::endl;
	std::cout<<"Initial cost is "<<C.sum()<<std::endl;
	std::cout<<"Press any key to print initial x and u to file..."<<std::endl;
	std::cin.get();
	write2file_eigen(x,"x");
	write2file_eigen(u,"u");
	std::cout<<"Please use python script to plot figures"<<std::endl;
	std::cout<<"Press any key to continue..."<<std::endl;
	std::cin.get();
}

Eigen::MatrixXd DDP::LQRdynamics(Eigen::MatrixXd x_i, Eigen::MatrixXd u_in_dynamics)
{
	Eigen::MatrixXd x_in_dynamics;
	x_in_dynamics.setZero(x_dim,T);
	x_in_dynamics.col(0) = x_i;
	WorldPtr DARTWorld_in_dynamics = mDDPWorld->clone();
	SkeletonPtr mCartPole_in_dynamics = DARTWorld_in_dynamics->getSkeleton("mCartPole");
	mCartPole_in_dynamics->getDof("Joint_hold_cart")->setPosition(x_i(0));
	mCartPole_in_dynamics->getDof("Joint_cart_pole")->setPosition(x_i(1));
	mCartPole_in_dynamics->getDof("Joint_hold_cart")->setVelocity(x_i(2));
	mCartPole_in_dynamics->getDof("Joint_cart_pole")->setVelocity(x_i(3));

	for (int i=0; i<T-1; i++)
	{
		DARTWorld_in_dynamics->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setForce(u_in_dynamics.col(i)(0));	

		DARTWorld_in_dynamics->step();

		x_in_dynamics(0,i+1) = mCartPole_in_dynamics->getDof("Joint_hold_cart")->getPosition();
		x_in_dynamics(1,i+1) = mCartPole_in_dynamics->getDof("Joint_cart_pole")->getPosition();
		x_in_dynamics(2,i+1) = mCartPole_in_dynamics->getDof("Joint_hold_cart")->getVelocity();
		x_in_dynamics(3,i+1) = mCartPole_in_dynamics->getDof("Joint_cart_pole")->getVelocity();
	}
	
	return x_in_dynamics;
}


void DDP::trajopt()
{
// one iteration of DDP
// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------
// backward pass
	bool diverge = true;
	while (diverge)
	{
		diverge = backwardpass();
		if (diverge)
		{
			mu +=5;
		}
	}
	mu = 1;
// -----------------------------------------------------------------------------------------------
//  backward debugging
		std::cout<<diverge<<std::endl;
		std::cout<<"Press any key to print k, K, Vx, Vxx to file..."<<std::endl;
		std::cin.get();
		write2file_std(k,"k");
		write2file_std(K,"K");
		write2file_std(Vx,"Vx");
		write2file_std(Vxx,"Vxx");
// -----------------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------
// forward  pass
	bool forward_done = false;
	while(!forward_done)
	{
		forwardpass();

		std::cout<<"One forward iteration finishes..."<<std::endl;
		std::cout<<"Press any key to continue..."<<std::endl;
		std::cin.get();
		//std::cout<<C.transpose()<<std::endl;
		//std::cout<<C_new.transpose()<<std::endl;

		double dCost = C.sum() - C_new.sum();
		double expected = -alpha*(dV[0]+alpha*dV[1]);
		double z;
		if (expected>0)
		{
			z = dCost/expected;
			std::cout<<"dCost: "<<dCost<<" expected: "<<expected<<std::endl;
			dtmsg<<"positive expected reduction "<<z<<std::endl;
			std::cout<<"Press any key to continue..."<<std::endl;
			std::cin.get();
		}
		else
		{
			z = 2*(dCost > 0)-1;
			dtmsg<<"non-positive expected reduction "<<z<<std::endl;
			std::cout<<"Press any key to continue..."<<std::endl;
			std::cin.get();
		}
		if (z>0)
		{
			forward_done = true;
			break;
		}
		else
		{
			alpha *=0.1;
		}
	}
	alpha = 1;
//------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------
	x = x_new;
	u = u_new;
	C = C_new;
//------------------------------------------------------------------------------------------------
	std::cout<<"Current cost is "<<C.sum()<<std::endl;
	std::cout<<"Press any key to print x and u to file..."<<std::endl;
	std::cin.get();
	write2file_eigen(x,"x");
	write2file_eigen(u,"u");
	std::cout<<"Please use python script to plot figures"<<std::endl;
	std::cout<<"Press any key to continue..."<<std::endl;
	std::cin.get();
}

bool DDP::backwardpass()
{
//  variable initialization
	dtmsg<<"Backward pass starting..."<<std::endl;
	bool localDiverge = false;
	dV.setZero();
	for (int i=0;i<T;i++)
	{
		Vx[i].setZero();
		Vxx[i].setZero();
		k[i].setZero();
		K[i].setZero();
	}

	for (int i=T-1;i>=0;i--)
	{
		LQRderivative(x.col(i),u.col(i));
		if (i==T-1)
		{
			Vx[i]  = Cx;
			Vxx[i] = Cxx;
			continue;
		}
		Eigen::Matrix<double,1,x_dim>	  Qx;	
		Eigen::Matrix<double,1,u_dim>	  Qu;
		Eigen::Matrix<double,x_dim,x_dim> Qxx;
		Eigen::Matrix<double,u_dim,u_dim> Quu;
		Eigen::Matrix<double,u_dim,x_dim> Qux;
		Eigen::Matrix<double,u_dim,u_dim> Quu_reg;
		Eigen::Matrix<double,u_dim,x_dim> Qux_reg;
		Qx		= Cx + Vx[i+1]*fx;
		Qu		= Cu + Vx[i+1]*fu;
		Qxx		= Cxx + fx.transpose()*Vxx[i+1]*fx;
		Quu		= Cuu + fu.transpose()*Vxx[i+1]*fu;
		Qux		= Cux + fu.transpose()*Vxx[i+1]*fx;
		Quu_reg = Quu + mu*Eigen::Matrix<double,u_dim,u_dim>::Identity();
		Qux_reg = Qux;
//----------------------------------------------------------------------------------------------------------------------------------
//      backward debugging
		if (i%100 == 0)
		{
			dtmsg<<" "<<i<<" step in backward pass"<<std::endl;
			std::cout<<"***************************************"<<std::endl;
			std::cout<<"fx "<<std::endl<<fx<<std::endl;
			std::cout<<"***************************************"<<std::endl;
			std::cout<<"fu "<<std::endl<<fu<<std::endl;
			std::cout<<"***************************************"<<std::endl;
			std::cout<<"Cx "<<std::endl<<Cx<<std::endl;
			std::cout<<"***************************************"<<std::endl;
			std::cout<<"Cu "<<std::endl<<Cu<<std::endl;
			std::cout<<"***************************************"<<std::endl;
			std::cout<<"Cxx "<<std::endl<<Cxx<<std::endl;
			std::cout<<"***************************************"<<std::endl;
			std::cout<<"Cuu "<<std::endl<<Cuu<<std::endl;
			std::cout<<"***************************************"<<std::endl;
			std::cout<<"Cux "<<std::endl<<Cux<<std::endl;
			std::cout<<"***************************************"<<std::endl;
			std::cout<<"Qx "<<std::endl<<Qx<<std::endl;
			std::cout<<"***************************************"<<std::endl;
			std::cout<<"Qu "<<std::endl<<Qu<<std::endl;
			std::cout<<"***************************************"<<std::endl;
			std::cout<<"Qxx "<<std::endl<<Qxx<<std::endl;
			std::cout<<"***************************************"<<std::endl;
			std::cout<<"Quu "<<std::endl<<Quu<<std::endl;
			std::cout<<"***************************************"<<std::endl;
			std::cout<<"Quu_reg "<<std::endl<<Quu_reg<<std::endl;
			std::cout<<"***************************************"<<std::endl;
			std::cout<<"Qux_reg "<<std::endl<<Qux_reg<<std::endl;
			std::cout<<"***************************************"<<std::endl;
			std::cout<<"***************************************"<<std::endl;
			std::cout<<"value function derivative "<<std::endl;
			std::cout<<"Vx "<<std::endl<<Vx[i+1]<<std::endl;
			std::cout<<"***************************************"<<std::endl;
			std::cout<<"Vxx "<<std::endl<<Vxx[i+1]<<std::endl;
			std::cout<<"***************************************"<<std::endl;
			//std::cin.get();
		}
//----------------------------------------------------------------------------------------------------------------------------------
		if (Quu_reg(0)<=0)
		{
			localDiverge = true;
			dtmsg<<"Diverge occurs in the backward pass"<<std::endl;
			return localDiverge;
		}
		k[i]	= -Qu/Quu_reg(0);
		K[i]	- -Qux_reg/Quu_reg(0);

		dV	   += (Eigen::Matrix<double,1,x_dim>()<< k[i].transpose()*Qu, 1/2*k[i].transpose()*Quu*k[i]).finished();
		Vx[i]   = (Qx.transpose()+K[i].transpose()*Quu*k[i]+K[i].transpose()*Qu+Qux.transpose()*k[i]).transpose();
		Vxx[i]  = Qxx + K[i].transpose()*Quu*K[i]+K[i].transpose()*Qux+Qux.transpose()*K[i];
		Vxx[i]  = 0.5*(Vxx[i]+Vxx[i].transpose());
	}
	return localDiverge;
}

void DDP::forwardpass()
{
	dtmsg<<"Forward pass starting..."<<std::endl;
	x_new.setZero();
	u_new.setZero();
	C_new.setZero();
//  x_new initial state shoud be (0,0,0,0).(transpose)
	{
		// reset x to zero
		// When clone the world, x is automatically reset to 0
		WorldPtr DARTdynamicsWorld = mDDPWorld->clone();
		for (int i=0; i<T-1; i++)
		{
			u_new.col(i) = u.col(i) + alpha*k[i] + K[i]*(x_new.col(i)-x.col(i));
			DARTdynamicsWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setForce(u_new.col(i)[0]);	
			DARTdynamicsWorld->step();

			x_new(0,i+1) = DARTdynamicsWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->getPosition();
			x_new(1,i+1) = DARTdynamicsWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->getPosition();
			x_new(2,i+1) = DARTdynamicsWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->getVelocity();
			x_new(3,i+1) = DARTdynamicsWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->getVelocity();
			C_new[i]=LQRcost(x_new.col(i),u_new.col(i));

//----------------------------------------------------------------------------------------------- 
// debug C_new cost
//			std::cout<<"x_new: "<<x_new.col(i).transpose()<<" u_new: "<<u_new.col(i)<<std::endl;
//			std::cout<<cost(x_new.col(i),u_new.col(i))<<std::endl;;
//			std::cout<<C_new[i]<<std::endl;
//			std::cout<<"Press any key to continue..."<<std::endl;
//			std::cin.get();
//----------------------------------------------------------------------------------------------- 
		}
		u_new.col(T-1) = Eigen::VectorXd::Constant(u_dim,std::nan("0"));
		C_new[T-1]=0.5*(x_new.col(T-1)-x_f).transpose()*Qf*(x_new.col(T-1)-x_f);
	}
}

void DDP::LQRderivative(Eigen::Vector4d x_i, Eigen::Matrix<double,1,1> u_i)
{
// compute fx, fu, Cx, Cu, Cxx, Cuu, Cux according to x_i and u_i
// Cx, Cu, Cxx, Cuu, Cux are computed according to analytic solution
// whether has delta_t here, I think it shouldn't. Because all we consider
// is discreteized system.
	Cx = (Q*(x_i - x_f)).transpose();
	Cu = (R*u_i).transpose();
	Cxx = Q;
	Cuu = R;
	Cux = Eigen::Matrix<double,u_dim,x_dim>::Zero();
	
// fx, fu are computed according to finite difference
	Eigen::Vector4d f_xt_ut;
	{
		WorldPtr DARTderivativeWorld = mDDPWorld->clone();
		
		DARTderivativeWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setPosition(x_i(0));
		DARTderivativeWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->setPosition(x_i(1));
		DARTderivativeWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setVelocity(x_i(2));
		DARTderivativeWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->setVelocity(x_i(3));

		DARTderivativeWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setForce(u_i(0));	
		DARTderivativeWorld->step();

		f_xt_ut(0) = DARTderivativeWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->getPosition();
		f_xt_ut(1) = DARTderivativeWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->getPosition();
		f_xt_ut(2) = DARTderivativeWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->getVelocity();
		f_xt_ut(3) = DARTderivativeWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->getVelocity();
	}
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

double DDP::LQRcost(Eigen::Vector4d x_i, Eigen::Matrix<double,1,1> u_i)
{
	double result = 0;
	result		 += 0.5*(x_i - x_f).transpose()*Q*(x_i-x_f);
	result		 += 0.5*u_i.transpose()*R*u_i;
	return result;
}

template<typename dataFormat_std>
void DDP::write2file_std(dataFormat_std data, const std::string name)
{
	std::string name_ext = name;
	name_ext.append(".out");
	std::ofstream outFile(name_ext, std::ios::out);
	if (outFile.fail())
	{
		dtmsg << "Cannot open "<<name<<" file, please check..."<<std::endl;
	}
	outFile.precision(8);
	for (size_t i=0; i<data.size(); i++)
	{
		outFile<<data[i]<<std::endl;
	}
	outFile.close();
}

template<typename dataFormat_eigen>
void DDP::write2file_eigen(dataFormat_eigen data, const std::string name)
{
	std::string name_ext = name;
	name_ext.append(".out");
	std::ofstream outFile(name_ext, std::ios::out);
	if (outFile.fail())
	{
		dtmsg << "Cannot open "<<name<<" file, please check..."<<std::endl;
	}
	outFile.precision(8);
	outFile<<data<<std::endl;
	outFile.close();
}
