#ifndef DDP_H
#define DDP_H 

#include <vector>
#include <cmath>
#include <string>
#include <fstream>
#include <functional>
#include "dart/dart.h"

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
	DDP(int T, WorldPtr mDDPWorld, std::function<Eigen::VectorXd(const Eigen::VectorXd, const Eigen::VectorXd)> StepDynamics, std::function<double(const Eigen::VectorXd, const Eigen::VectorXd)> StepCost, std::function<double(const Eigen::VectorXd)> FinalCost);
	void LQRderivative(Eigen::Vector4d x_i, Eigen::Matrix<double,1,1> u_i);

	Eigen::MatrixXd TrajGenerator(const Eigen::VectorXd _x0, const Eigen::MatrixXd _u);
	std::function<Eigen::VectorXd(const Eigen::VectorXd, const Eigen::VectorXd)> StepDynamics;
	std::function<double(const Eigen::VectorXd, const Eigen::VectorXd)> StepCost;
	std::function<double(const Eigen::VectorXd)> FinalCost;
// --------------------------------------------------
	template<typename dataFormat_std>
	void write2file_std(dataFormat_std data, const std::string name);
	template<typename dataFormat_eigen>
	void write2file_eigen(dataFormat_eigen data, const std::string name);
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
	std::vector<Eigen::Matrix<double,1,x_dim>> Vx;
	std::vector<Eigen::Matrix<double,x_dim,x_dim>> Vxx;
	Eigen::Matrix<double,1,x_dim> dV;
	std::vector<Eigen::Matrix<double,1,u_dim>> k;
	std::vector<Eigen::Matrix<double,1,x_dim>> K;
	Eigen::Matrix<double,x_dim,x_dim> fx;
	Eigen::Matrix<double,x_dim,u_dim> fu;
	Eigen::Matrix<double,1,x_dim>	  Cx;
	Eigen::Matrix<double,1,u_dim>	  Cu;
	Eigen::Matrix<double,x_dim,x_dim> Cxx;
	Eigen::Matrix<double,u_dim,u_dim> Cuu;
	Eigen::Matrix<double,u_dim,x_dim> Cux;
	double mu;
	double alpha;
// --------------------------------------------------
	double delta_t;
	WorldPtr mDDPWorld;
	double h;
// --------------------------------------------------
	Eigen::Matrix<double,x_dim,x_dim> Qf;
	Eigen::Matrix<double,x_dim,x_dim> Q;
	Eigen::Matrix<double,u_dim,u_dim> R;
	Eigen::Matrix<double,x_dim,1>     x_f;
	Eigen::Matrix<double,x_dim,1>	  x_0;
// --------------------------------------------------
	double m_c;
	double m_p;
	double l;
	double g;	
};

#endif
