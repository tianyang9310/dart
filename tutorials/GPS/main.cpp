#include <functional>
#include <valarray>
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

    mWorld->setTimeStep(0.02);

// ---------------------------------------------------------
// hyperparameter initialization
	int T				= 100;
    int x_dim           = 4;
    int u_dim           = 1;
	int numDDPIters		= 80;
	int conditions  	= 6;
	valarray<int> numSamplePerPolicy(conditions+1);
    // num of samples for each DDP policy
    numSamplePerPolicy[0] = 100;
    numSamplePerPolicy[1] = 100;
    numSamplePerPolicy[2] = 50;
    numSamplePerPolicy[3] = 50;
    numSamplePerPolicy[4] = 100;
    numSamplePerPolicy[5] = 100;
    // num of samples for each NN policy
    numSamplePerPolicy[6] = 10;


// ---------------------------------------------------------
	Vector4d x0			= Vector4d::Zero();
	Vector4d xd	        = Vector4d::Zero();
	xd(1)				= M_PI;
////////////////////////////////////////////////////////////
	double delta_tswing	= mWorld->getTimeStep();

	Matrix4d Qswing	= Matrix4d::Zero();
	Qswing(0,0)				= 0.01;
	Qswing(1,1)				= 5;
	Qswing(2,2)				= 1;
	Qswing(3,3)				= 1;

	Matrix<double,1,1> Rswing;
	Rswing(0,0)				= 1;

	Matrix4d Qfswing = Matrix4d::Identity();
	Qfswing(0,0)				= 1;
	Qfswing(1,1)				= 500;
	Qfswing(2,2)				= 100;
	Qfswing(3,3)				= 100;

	Qswing			= Qswing*delta_tswing;
	Rswing			= Rswing*delta_tswing;

	vector<tuple<MatrixXd, MatrixXd, MatrixXd>> LQRswing;

	LQRswing.push_back(make_tuple(Qswing,Rswing,Qfswing));
////////////////////////////////////////////////////////////
	double delta_tbalac	= mWorld->getTimeStep();

	Matrix4d Qbalac	= Matrix4d::Zero();
	Qbalac(0,0)				= 0.01;
	Qbalac(1,1)				= 5;
	Qbalac(2,2)				= 1;
	Qbalac(3,3)				= 1;

	Matrix<double,1,1> Rbalac;
	Rbalac(0,0)				= 1;

	Matrix4d Qfbalac = Matrix4d::Identity();
	Qfbalac(0,0)				= 1;
	Qfbalac(1,1)				= 500;
	Qfbalac(2,2)				= 100;
	Qfbalac(3,3)				= 100;

    delta_tbalac = 100;
	Qbalac			= Qbalac*delta_tbalac;
	// Rbalac			= Rbalac*delta_tbalac;

	vector<tuple<MatrixXd, MatrixXd, MatrixXd>> LQRbalac;

	LQRbalac.push_back(make_tuple(Qbalac,Rbalac,Qfbalac));
// ---------------------------------------------------------
	unique_ptr<GPS> mGPS = unique_ptr<GPS>(new GPS(T,x_dim,u_dim,numDDPIters,conditions,numSamplePerPolicy,bind(DartStepDynamics, placeholders::_1, placeholders::_2, mWorld->clone())));

	for_each(mGPS->x0Bundle.begin(),mGPS->x0Bundle.end(),[x0](VectorXd& x0Bundle_ind){x0Bundle_ind=x0;});

    mGPS->x0Bundle[0](1)=      0.0*M_PI;

	mGPS->x0Bundle[1](1)=      2.0*M_PI;

	mGPS->x0Bundle[2](1)=      0.9*M_PI;
	mGPS->x0Bundle[2](2)=      1.0*M_PI;
	mGPS->x0Bundle[2](3)=      2*M_PI;

	mGPS->x0Bundle[3](1)=      1.1*M_PI;
	mGPS->x0Bundle[3](2)=      -1.0*M_PI;
	mGPS->x0Bundle[3](3)=      -2*M_PI;

	mGPS->x0Bundle[4](1)=      0.65*M_PI;
	mGPS->x0Bundle[4](3)=      -0.2*M_PI;

	mGPS->x0Bundle[5](1)=      1.35*M_PI;
	mGPS->x0Bundle[5](3)=      0.2*M_PI;

	mGPS->DDPBundle[0] = make_shared<DDP>(DDP(T, bind(DartStepDynamics, placeholders::_1, placeholders::_2, mWorld->clone()), bind(CartPoleStepCost, placeholders::_1, placeholders::_2, xd, Qswing, Rswing), bind(CartPoleFinalCost,placeholders::_1, xd, Qfswing), LQRswing, make_tuple(mGPS->x0Bundle[0],xd,1)));
	mGPS->DDPBundle[1] = make_shared<DDP>(DDP(T, bind(DartStepDynamics, placeholders::_1, placeholders::_2, mWorld->clone()), bind(CartPoleStepCost, placeholders::_1, placeholders::_2, xd, Qswing, Rswing), bind(CartPoleFinalCost,placeholders::_1, xd, Qfswing), LQRswing, make_tuple(mGPS->x0Bundle[1],xd,1)));
	mGPS->DDPBundle[2] = make_shared<DDP>(DDP(T, bind(DartStepDynamics, placeholders::_1, placeholders::_2, mWorld->clone()), bind(CartPoleStepCost, placeholders::_1, placeholders::_2, xd, Qswing, Rswing), bind(CartPoleFinalCost,placeholders::_1, xd, Qfswing), LQRswing, make_tuple(mGPS->x0Bundle[2],xd,1)));
	mGPS->DDPBundle[3] = make_shared<DDP>(DDP(T, bind(DartStepDynamics, placeholders::_1, placeholders::_2, mWorld->clone()), bind(CartPoleStepCost, placeholders::_1, placeholders::_2, xd, Qswing, Rswing), bind(CartPoleFinalCost,placeholders::_1, xd, Qfswing), LQRswing, make_tuple(mGPS->x0Bundle[3],xd,1)));
	mGPS->DDPBundle[4] = make_shared<DDP>(DDP(T, bind(DartStepDynamics, placeholders::_1, placeholders::_2, mWorld->clone()), bind(CartPoleStepCost, placeholders::_1, placeholders::_2, xd, Qswing, Rswing), bind(CartPoleFinalCost,placeholders::_1, xd, Qfswing), LQRswing, make_tuple(mGPS->x0Bundle[4],xd,1)));
	mGPS->DDPBundle[5] = make_shared<DDP>(DDP(T, bind(DartStepDynamics, placeholders::_1, placeholders::_2, mWorld->clone()), bind(CartPoleStepCost, placeholders::_1, placeholders::_2, xd, Qswing, Rswing), bind(CartPoleFinalCost,placeholders::_1, xd, Qfswing), LQRswing, make_tuple(mGPS->x0Bundle[5],xd,1)));

// ---------------------------------------------------------

	MyWindow window(mWorld,move(mGPS));

	glutInit(&argc, argv);
	window.initWindow(1024, 768, "Guided Policy Search");
	glutMainLoop();

	return 0;
}
