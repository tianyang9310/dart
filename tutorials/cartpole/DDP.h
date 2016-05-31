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


class DDP
{
public:
	void trajopt();
	bool backwardpass();
	void forwardpass();
// --------------------------------------------------
	DDP(int T, WorldPtr mDDPWorld, std::function<Eigen::VectorXd(const Eigen::VectorXd, const Eigen::VectorXd)> StepDynamics, std::function<Scalar(const Eigen::VectorXd, const Eigen::VectorXd)> StepCost, std::function<Scalar(const Eigen::VectorXd)> FinalCost, std::vector<std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd>> LQR);
	void Derivative(Eigen::VectorXd _xi, Eigen::VectorXd _ui);

	Eigen::MatrixXd TrajGenerator(const Eigen::VectorXd _x0, const Eigen::MatrixXd _u);


	std::function<Eigen::VectorXd(const Eigen::VectorXd, const Eigen::VectorXd)> StepDynamics;
	std::function<Scalar(const Eigen::VectorXd, const Eigen::VectorXd)> StepCost;
	std::function<Scalar(const Eigen::VectorXd)> FinalCost;
// --------------------------------------------------
	const int T;
	static const int x_dim = 4;
	static const int u_dim = 1;
	Eigen::MatrixXd x;
	Eigen::MatrixXd u;
	Eigen::VectorXd C;
	Eigen::MatrixXd x_new;
	Eigen::MatrixXd u_new;
	Eigen::VectorXd C_new;
	std::vector<Eigen::Matrix<double,x_dim,1>> Vx;
	std::vector<Eigen::Matrix<double,x_dim,x_dim>> Vxx;
	Eigen::Vector2d dV;
	std::vector<Eigen::Matrix<double,u_dim,1>> k;
	std::vector<Eigen::Matrix<double,u_dim,x_dim>> K;
	Eigen::Matrix<double,x_dim,x_dim> fx;
	Eigen::Matrix<double,x_dim,u_dim> fu;
	Eigen::Matrix<double,x_dim,1>	  Cx;
	Eigen::Matrix<double,u_dim,1>	  Cu;
	Eigen::Matrix<double,x_dim,x_dim> Cxx;
	Eigen::Matrix<double,u_dim,u_dim> Cuu;
	Eigen::Matrix<double,u_dim,x_dim> Cux;
	double mu;
	double alpha;
// --------------------------------------------------
	WorldPtr mDDPWorld;
	double h;
// --------------------------------------------------
	Eigen::Matrix<double,x_dim,x_dim> Qf;
	Eigen::Matrix<double,x_dim,x_dim> Q;
	Eigen::Matrix<double,u_dim,u_dim> R;
	Eigen::Matrix<double,x_dim,1>     x_f;
	Eigen::Matrix<double,x_dim,1>	  x_0;
// --------------------------------------------------
	bool isLQR;
};


template<typename FuncType>
Eigen::MatrixXd FiniteDiff(FuncType Func, Eigen::VectorXd Var)
{
	// Note:
	//		1. The order of mixed partial derivatives of Hessian
	//			Determine whether it's hessian via the dimensions of output
	//		2. This finite difference function can return Jacobian
	//			In other words, if the output is 1 by 1 and input is x by 1, 
	//			then it returns 1 by x. If the output is x by 1 and input is x by 1,
	//			then it returns x by x
	//		3. Therefore for the output of single value function, extra effort is needed
	//			to make sure the derivative is x by 1. (The returned one is 1 by x)
	//		4. No need to be a member function
	double h = 1e-6;
	int VarDim = Var.rows();
	int OutputDim;
	bool Hessian;
	Eigen::MatrixXd J_FD;
	if ((Func(Var)).cols() == 1)
	{
		OutputDim = (Func(Var)).rows();
		Hessian	  = false;
		J_FD.resize(OutputDim,VarDim);
	}
	else
	{
		OutputDim = (Func(Var)).cols();
		Hessian	  = true;
		J_FD.resize(VarDim,OutputDim);
	}

	Eigen::MatrixXd IndicatorVec(Var.rows(),Var.rows());
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
	return J_FD;
}


// --------------------------------------------------
template<typename dataFormat_std>
void write2file_std(dataFormat_std data, const std::string name)
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
void write2file_eigen(dataFormat_eigen data, const std::string name)
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


#endif
