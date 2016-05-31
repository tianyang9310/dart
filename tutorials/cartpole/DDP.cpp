#include "DDP.h"

DDP::DDP(int T, WorldPtr mDDPWorld, std::function<Eigen::VectorXd(const Eigen::VectorXd, const Eigen::VectorXd)> StepDynamics, std::function<Scalar(const Eigen::VectorXd, const Eigen::VectorXd)> StepCost, std::function<Scalar(const Eigen::VectorXd)> FinalCost, std::vector<std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd>> LQR):
	T(T),
	Vx(T),
	Vxx(T),
	k(T),
	K(T),
	mDDPWorld(mDDPWorld),
	StepDynamics(StepDynamics),
	StepCost(StepCost),
	FinalCost(FinalCost)
{
// --------------------------------------------------
// constant initialization
	isLQR		= false;
	if (!LQR.empty())
	{
		isLQR	= true;
		Q		= std::get<0>(LQR[0]);
		R		= std::get<1>(LQR[0]);
		Qf		= std::get<2>(LQR[0]);
	}

	x_f.setZero();
	x_f(1)      = M_PI;
	x_0.setZero();
	mu    = 0;
	alpha = 1;
	h     = 1e-6;

// --------------------------------------------------
// produce initial trajectory
	u 		= Eigen::MatrixXd::Constant(u_dim,T,0);
	//u 		= Eigen::MatrixXd::Random(u_dim,T)*150;
	u.col(T-1) = Eigen::VectorXd::Constant(u_dim,std::nan("0"));

	x	    = Eigen::MatrixXd::Zero(x_dim,T);
// --------------------------------------------------
	x		= TrajGenerator(x_0, u);
// --------------------------------------------------
	C		= Eigen::VectorXd::Zero(T);
// --------------------------------------------------
	x_new   = Eigen::MatrixXd::Zero(x_dim,T);
	u_new   = Eigen::MatrixXd::Zero(u_dim,T);
	C_new	= Eigen::VectorXd::Zero(T);
// --------------------------------------------------
	dV.setZero();
	for (int i=0;i<T;i++)
	{
		Vx[i].setZero();
		Vxx[i].setZero();
		k[i].setZero();
		K[i].setZero();
		if (i<T-1)
		{
			C.col(i) = StepCost(x.col(i),u.col(i));
		}
	}
	C.col(T-1)  = FinalCost(x.col(T-1));

// --------------------------------------------------
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

// --------------------------------------------------
// DDP initial data and some variable output
//	std::cout<<"Initial control sequence is"<<std::endl<<u<<std::endl;
//	std::cout<<"Initial state   sequence is"<<std::endl<<x.transpose()<<std::endl;
	std::cout<<"Initial cost is "<<C.sum()<<std::endl;
	std::cout<<"Press any key to print initial x and u to file..."<<std::endl;
//	std::cin.get();
	write2file_eigen(x,"x");
	write2file_eigen(u,"u");
	std::cout<<"Please use python script to plot figures"<<std::endl;
	std::cout<<"Press any key to continue..."<<std::endl;
	//std::cin.get();
}

Eigen::MatrixXd DDP::TrajGenerator(const Eigen::VectorXd _x0, const Eigen::MatrixXd _u)
{
// --------------------------------------------------
//	Whole Trajectory Generator
//	Computing _x according to _x0 and _u using step dynamics
// --------------------------------------------------

	Eigen::MatrixXd _x;
	_x.setZero(_x0.rows(),T);
	_x.col(0) = _x0;

	for (int i=0; i<T-1; i++)
	{
		_x.col(i+1) = StepDynamics(_x.col(i), _u.col(i));
	}
	
	return _x;
}

void DDP::trajopt()
{
// one iteration of DDP
// --------------------------------------------------
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
	mu = 0;
// --------------------------------------------------
//  backward debugging
		std::cout<<diverge<<std::endl;
		std::cout<<"Press any key to print k, K, Vx, Vxx to file..."<<std::endl;
		write2file_std(k,"k");
		write2file_std(K,"K");
		write2file_std(Vx,"Vx");
		write2file_std(Vxx,"Vxx");
		std::cin.get();
// --------------------------------------------------

// forward  pass
	bool forward_done = false;
	while(!forward_done)
	{
		forwardpass();

		std::cout<<"One forward iteration finishes..."<<std::endl;
		std::cout<<"Press any key to continue..."<<std::endl;
	//	std::cin.get();
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
	//		std::cin.get();
		}
		else
		{
			z = 2*(dCost > 0)-1;
			dtmsg<<"non-positive expected reduction "<<z<<std::endl;
			std::cout<<"Press any key to continue..."<<std::endl;
	//		std::cin.get();
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
// --------------------------------------------------
	x = x_new;
	u = u_new;
	C = C_new;
// --------------------------------------------------
	std::cout<<"Current cost is "<<C.sum()<<std::endl;
	std::cout<<"Press any key to print x and u to file..."<<std::endl;
	//std::cin.get();
	write2file_eigen(x,"x");
	write2file_eigen(u,"u");
	std::cout<<"Please use python script to plot figures"<<std::endl;
	std::cout<<"Press any key to continue..."<<std::endl;
	//std::cin.get();
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

	if (isLQR)
	{
		Vx[T-1]		= (Qf*(x.col(T-1)-x_f)).transpose();
		Vxx[T-1]	= Qf;
	}
	else
	{
	// use finite difference to compute Vx and Vxx
//		Cx = FiniteDiff([=](Eigen::VectorXd Var){
//				return StepDynamics(Var.head(_xi.rows()), Var.tail(_ui.rows()));}, 
//				(Eigen::VectorXd(_xi.rows()+_ui.rows()) << _xi, _ui).finished());
//		Vx[T-1]		= Cx;
//		Vxx[T-1]	= Cxx;
	}

	for (int i=T-2;i>=0;i--)
	{
		Derivative(x.col(i),u.col(i));

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
// --------------------------------------------------
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
			std::cout<<"Break point in backward pass. Press any key to continue"<<std::endl;
			//std::cin.get();
		}
// --------------------------------------------------
		if (Quu_reg(0)<=0)
		{
			localDiverge = true;
			dtmsg<<"Diverge occurs in the backward pass"<<std::endl;
			return localDiverge;
		}
		k[i]	= -Qu/Quu_reg(0);
		K[i]	= -Qux_reg/Quu_reg(0);

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
			C_new.col(i) = StepCost(x_new.col(i),u_new.col(i));

// --------------------------------------------------
// debug C_new cost
//			std::cout<<"x_new: "<<x_new.col(i).transpose()<<" u_new: "<<u_new.col(i)<<std::endl;
//			std::cout<<cost(x_new.col(i),u_new.col(i))<<std::endl;;
//			std::cout<<C_new[i]<<std::endl;
//			std::cout<<"Press any key to continue..."<<std::endl;
//			std::cin.get();
// --------------------------------------------------
		}
		u_new.col(T-1) = Eigen::VectorXd::Constant(u_dim,std::nan("0"));
		C_new.col(T-1) = FinalCost(x_new.col(T-1));
	}
}

void DDP::Derivative(Eigen::VectorXd _xi, Eigen::VectorXd _ui)
{
// compute fx, fu, Cx, Cu, Cxx, Cuu, Cux according to x_i and u_i
// Cx, Cu, Cxx, Cuu, Cux are computed according to analytic solution
// whether has delta_t here, I think it shouldn't. Because all we consider
// is discreteized system.
	if (isLQR)
	{
		Cx = (Q*(_xi - x_f)).transpose();
		Cu = (R*_ui).transpose();
		Cxx = Q;
		Cuu = R;
		Cux = Eigen::Matrix<double,u_dim,x_dim>::Zero();
	}
	else
	{
		// use finite difference to compute
	}
	
// fx, fu are computed according to finite difference
	Eigen::MatrixXd fxu;
	fxu = FiniteDiff([=](Eigen::VectorXd Var){
			return StepDynamics(Var.head(_xi.rows()), Var.tail(_ui.rows()));}, 
			(Eigen::VectorXd(_xi.rows()+_ui.rows()) << _xi, _ui).finished());
	fx  = fxu.leftCols(_xi.rows());
	fu	= fxu.rightCols(_ui.rows());
}

