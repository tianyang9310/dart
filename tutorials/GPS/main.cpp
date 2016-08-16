#include <functional>
#include <valarray>
#include "dart/dart.h"
#include "CartPole/WorldSetup.h"
#include "CartPole/CartPoleUtility.h"
#include "GUI/MyWindow.h"
#include "libGPS/GPS.h"
#include "../DDP/libDDP/DDP.h"
#include "../MPC/libMPC/MPC.h"

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
using namespace MPC_NSpace;
using namespace CartPoleUtility;

int main(int argc, char* argv[])
{
	WorldPtr mWorld = make_shared<World>();
	WorldSetup(mWorld);

    mWorld->setTimeStep(0.02);


// ---------------------------------------------------------
// hyperparameter initialization
	int T				= 100;
    int local_T         = 20;
    int x_dim           = 4;
    int u_dim           = 1;
	int numDDPIters		= 50;
	int conditions  	= 4;
	int CondOpt       	= 2;
	valarray<int> numSamplePerPolicy(conditions+1);
    numSamplePerPolicy[0] = 10;
    numSamplePerPolicy[1] = 10;
    numSamplePerPolicy[2] = 10;
    numSamplePerPolicy[3] = 10;
    numSamplePerPolicy[4] = 10;


	double delta_t		= mWorld->getTimeStep();
	Vector4d x0			= Vector4d::Zero();

	Vector4d	xd	= Vector4d::Zero();
	xd(1)				= M_PI;

	Matrix4d Q	= Matrix4d::Zero();
	Q(0,0)				= 0.01;
	Q(1,1)				= 5;
	Q(2,2)				= 5;
	Q(3,3)				= 5;

	Matrix<double,1,1> R;
	R(0,0)				= 1;

	Matrix4d Qf = Matrix4d::Identity();
	Qf(0,0)				= 10;
	Qf(1,1)				= 500;
	Qf(2,2)				= 500;
	Qf(3,3)				= 500;

	Q			= Q*delta_t;
	R			= R*delta_t;

	vector<tuple<MatrixXd, MatrixXd, MatrixXd>> LQR;

	LQR.push_back(make_tuple(Q,R,Qf));

// ---------------------------------------------------------
	unique_ptr<GPS> mGPS = unique_ptr<GPS>(new GPS(T,x_dim,u_dim,numDDPIters,conditions,CondOpt,numSamplePerPolicy,bind(DartStepDynamics, placeholders::_1, placeholders::_2, mWorld->clone())));
    unique_ptr<MPC> mMPC;

	for_each(mGPS->x0Bundle.begin(),mGPS->x0Bundle.end(),[=](VectorXd& x0Bundle_sub){x0Bundle_sub=x0;});

    mGPS->x0Bundle[0](1)=      0.0*M_PI;

	mGPS->x0Bundle[1](1)=      2.0*M_PI;

	mGPS->x0Bundle[2](1)= (1-1e-1)*M_PI;

	mGPS->x0Bundle[3](1)= (1+1e-1)*M_PI;

	mGPS->DDPBundle[0] = make_shared<DDP>(DDP(T, bind(DartStepDynamics, placeholders::_1, placeholders::_2, mWorld->clone()), bind(CartPoleStepCost, placeholders::_1, placeholders::_2, xd, Q, R), bind(CartPoleFinalCost,placeholders::_1, xd, Qf), LQR, make_tuple(mGPS->x0Bundle[0],xd,1)));
	mGPS->DDPBundle[1] = make_shared<DDP>(DDP(T, bind(DartStepDynamics, placeholders::_1, placeholders::_2, mWorld->clone()), bind(CartPoleStepCost, placeholders::_1, placeholders::_2, xd, Q, R), bind(CartPoleFinalCost,placeholders::_1, xd, Qf), LQR, make_tuple(mGPS->x0Bundle[1],xd,1)));


    int x0Bundle_idx;
////////////////////////////////////////////////////////////
    x0Bundle_idx = 2;
    mMPC = unique_ptr<MPC>(new MPC(local_T,T,x_dim,u_dim,mGPS->x0Bundle[x0Bundle_idx],xd,Q,R,Qf, bind(DartStepDynamics, placeholders::_1, placeholders::_2, mWorld->clone()), bind(CartPoleStepCost, placeholders::_1, placeholders::_2, xd, Q, R), bind(CartPoleFinalCost,placeholders::_1, xd, Qf)));
    mMPC->run();

	mGPS->DDPBundle[x0Bundle_idx] = make_shared<DDP>(DDP(T, bind(DartStepDynamics, placeholders::_1, placeholders::_2, mWorld->clone()), bind(CartPoleStepCost, placeholders::_1, placeholders::_2, xd, Q, R), bind(CartPoleFinalCost,placeholders::_1, xd, Qf), LQR, make_tuple(mGPS->x0Bundle[x0Bundle_idx],xd,1)));
    mGPS->DDPBundle[x0Bundle_idx]->u = mMPC->getU();
    mGPS->DDPBundle[x0Bundle_idx]->x = mGPS->DDPBundle[x0Bundle_idx]->TrajGenerator(mGPS->x0Bundle[x0Bundle_idx], mGPS->DDPBundle[x0Bundle_idx]->u);

////////////////////////////////////////////////////////////
    x0Bundle_idx = 3;
    mMPC = unique_ptr<MPC>(new MPC(local_T,T,x_dim,u_dim,mGPS->x0Bundle[x0Bundle_idx],xd,Q,R,Qf, bind(DartStepDynamics, placeholders::_1, placeholders::_2, mWorld->clone()), bind(CartPoleStepCost, placeholders::_1, placeholders::_2, xd, Q, R), bind(CartPoleFinalCost,placeholders::_1, xd, Qf)));
    mMPC->run();

	mGPS->DDPBundle[x0Bundle_idx] = make_shared<DDP>(DDP(T, bind(DartStepDynamics, placeholders::_1, placeholders::_2, mWorld->clone()), bind(CartPoleStepCost, placeholders::_1, placeholders::_2, xd, Q, R), bind(CartPoleFinalCost,placeholders::_1, xd, Qf), LQR, make_tuple(mGPS->x0Bundle[x0Bundle_idx],xd,1)));
    mGPS->DDPBundle[x0Bundle_idx]->u = mMPC->getU();
    mGPS->DDPBundle[x0Bundle_idx]->x = mGPS->DDPBundle[x0Bundle_idx]->TrajGenerator(mGPS->x0Bundle[x0Bundle_idx], mGPS->DDPBundle[x0Bundle_idx]->u);

// ---------------------------------------------------------

	MyWindow window(mWorld,move(mGPS));

	glutInit(&argc, argv);
	window.initWindow(1024, 768, "Guided Policy Search");
	glutMainLoop();

	return 0;
}
