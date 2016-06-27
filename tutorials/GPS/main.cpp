#include <functional>
#include "dart/dart.h"
#include "CartPole/WorldSetup.h"
#include "CartPole/CartPoleUtility.h"
#include "GUI/MyWindow.h"
#include "libGPS/GPS.h"
#include "../DDP/libDDP/DDP.h"

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;
using namespace dart::common;
using namespace dart::math;
using namespace dart::gui;
using namespace std;
using namespace Eigen;
using namespace GPS_NSpace;
using namespace DDP_NSpace;
using namespace CartPoleUtility;

int main(int argc, char* argv[])
{
	WorldPtr mWorld = make_shared<World>();
	WorldSetup(mWorld);


// ---------------------------------------------------------
// hyperparameter initialization
	int T				= 2000;
    int x_dim           = 4;
    int u_dim           = 1;
	int numDDPIters		= 3;
	int conditions  	= 5;
	int numSamplePerCond= 2;

	double delta_t		= mWorld->getTimeStep();
	Vector4d x0			= Vector4d::Zero();

	Vector4d	xd	= Vector4d::Zero();
	xd(1)				= M_PI;

	Matrix4d Q	= Matrix4d::Zero();
	Q(0,0)				= 0.01;
	Q(1,1)				= 5;

	Matrix<double,1,1> R;
	R(0,0)				= 1;

	Matrix4d Qf = Matrix4d::Identity();
	Qf(1,1)				= 500;

	Q			= Q*delta_t;
	R			= R*delta_t;

	vector<tuple<MatrixXd, MatrixXd, MatrixXd>> LQR;

	LQR.push_back(make_tuple(Q,R,Qf));

// ---------------------------------------------------------
	unique_ptr<GPS> mGPS = unique_ptr<GPS>(new GPS(T,x_dim,u_dim,numDDPIters,conditions,numSamplePerCond,bind(DartStepDynamics, placeholders::_1, placeholders::_2, mWorld->clone())));

	for_each(mGPS->x0Bundle.begin(),mGPS->x0Bundle.end(),[=](VectorXd& x0Bundle_sub){x0Bundle_sub=x0;});
	mGPS->x0Bundle[0](1)=-1/3.0*M_PI;
	mGPS->x0Bundle[1](1)=-1/6.0*M_PI;
	mGPS->x0Bundle[2](1)=   0.0*M_PI;
	mGPS->x0Bundle[3](1)= 1/6.0*M_PI;
	mGPS->x0Bundle[4](1)= 1/3.0*M_PI;

	mGPS->DDPBundle[0] = make_shared<DDP>(DDP(T, bind(DartStepDynamics, placeholders::_1, placeholders::_2, mWorld->clone()), bind(CartPoleStepCost, placeholders::_1, placeholders::_2, xd, Q, R), bind(CartPoleFinalCost,placeholders::_1, xd, Qf), LQR, make_tuple(mGPS->x0Bundle[0],xd,1)));
	mGPS->DDPBundle[1] = make_shared<DDP>(DDP(T, bind(DartStepDynamics, placeholders::_1, placeholders::_2, mWorld->clone()), bind(CartPoleStepCost, placeholders::_1, placeholders::_2, xd, Q, R), bind(CartPoleFinalCost,placeholders::_1, xd, Qf), LQR, make_tuple(mGPS->x0Bundle[1],xd,1)));
	mGPS->DDPBundle[2] = make_shared<DDP>(DDP(T, bind(DartStepDynamics, placeholders::_1, placeholders::_2, mWorld->clone()), bind(CartPoleStepCost, placeholders::_1, placeholders::_2, xd, Q, R), bind(CartPoleFinalCost,placeholders::_1, xd, Qf), LQR, make_tuple(mGPS->x0Bundle[2],xd,1)));
	mGPS->DDPBundle[3] = make_shared<DDP>(DDP(T, bind(DartStepDynamics, placeholders::_1, placeholders::_2, mWorld->clone()), bind(CartPoleStepCost, placeholders::_1, placeholders::_2, xd, Q, R), bind(CartPoleFinalCost,placeholders::_1, xd, Qf), LQR, make_tuple(mGPS->x0Bundle[3],xd,1)));
	mGPS->DDPBundle[4] = make_shared<DDP>(DDP(T, bind(DartStepDynamics, placeholders::_1, placeholders::_2, mWorld->clone()), bind(CartPoleStepCost, placeholders::_1, placeholders::_2, xd, Q, R), bind(CartPoleFinalCost,placeholders::_1, xd, Qf), LQR, make_tuple(mGPS->x0Bundle[4],xd,1)));

// ---------------------------------------------------------

	MyWindow window(mWorld,move(mGPS));

	glutInit(&argc, argv);
	window.initWindow(1024, 768, "Guided Policy Search");
	glutMainLoop();

	return 0;
}
