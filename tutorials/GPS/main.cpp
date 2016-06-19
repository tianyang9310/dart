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

	unique_ptr<GPS> mGPS = unique_ptr<GPS>(new GPS(20,5));

// ---------------------------------------------------------
// DDP initialization
	double delta_t = mWorld->getTimeStep();
	Vector4d x0  = Vector4d::Zero();

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

	mGPS->mDDP = unique_ptr<DDP>(new DDP(2000, bind(DartStepDynamics, placeholders::_1, placeholders::_2, mWorld->clone()), bind(CartPoleStepCost, placeholders::_1, placeholders::_2, xd, Q, R), bind(CartPoleFinalCost,placeholders::_1, xd, Qf), LQR, make_tuple(x0,xd,1)));

	for_each(mGPS->x0Bundle.begin(),mGPS->x0Bundle.end(),[=](Vector4d& x0Bundle_sub){x0Bundle_sub=x0;});
	mGPS->x0Bundle[0](1)=-1/3.0*M_PI;
	mGPS->x0Bundle[1](1)=-1/6.0*M_PI;
	mGPS->x0Bundle[2](1)=   0.0*M_PI;
	mGPS->x0Bundle[3](1)= 1/6.0*M_PI;
	mGPS->x0Bundle[4](1)= 1/3.0*M_PI;

// ---------------------------------------------------------

	MyWindow window(mWorld,move(mGPS));

	glutInit(&argc, argv);
	window.initWindow(1024, 768, "Guided Policy Search");
	glutMainLoop();

	return 0;
}
