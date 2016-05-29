/*************************************************************************
    > File Name: DDP.h
    > Author: Yang Tian
    > Mail: tianyang9310@gmail.com 
    > Created Time: Mon 23 May 2016 11:49:13 AM EDT
 ************************************************************************/

#ifndef DDP_H
#define DDP_H 

#include <vector>
#include <cmath>
#include <unistd.h>
#include <string>
#include <fstream>
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
	DDP(int T, WorldPtr mDDPWorld);
	Eigen::MatrixXd LQRdynamics(Eigen::MatrixXd x_i, Eigen::MatrixXd u_in_dynamics);
	double LQRcost(Eigen::Vector4d x_i, Eigen::Matrix<double,1,1> u_i);
	void LQRderivative(Eigen::Vector4d x_i, Eigen::Matrix<double,1,1> u_i);

	Eigen::MatrixXd dynamics(Eigen::MatrixXd x_i, Eigen::MatrixXd u_i);

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
