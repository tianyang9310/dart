/*************************************************************************
    > File Name: MyWindow.cpp > Author: Yang Tian
    > Mail: tianyang9310@gmail.com 
    > Created Time: Wed 16 Mar 2016 11:53:26 AM EDT
 ************************************************************************/

#include "MyWindow.h"
#include <time.h>
namespace toyexample{

MyWindow::MyWindow(WorldPtr world)
{
	setWorld(world);	

	mWorld->getConstraintSolver()->setCollisionDetector(new dart::collision::BulletCollisionDetector());
	detector = mWorld->getConstraintSolver()->getCollisionDetector();
	detector->detectCollision(true, true);

	mController =  std::unique_ptr<Controller>(new Controller(mWorld->getSkeleton("cube"), mWorld->getSkeleton("world_setup"), detector));

	mController->setCubeVelocity();

	mNewTimeStep = 0.01;
	mWorld->setTimeStep(mNewTimeStep);
	srand ((unsigned int)time(NULL));
}

void MyWindow::timeStepping() 
{
	if (mController->collisionEvent())
	{
		std::cout<<"collision detected!"<<std::endl;
	}

	// ideal condition is we don't need to set horizontal velocity at each timestep.
	// The ControlPBP can always lead to avoiding obstacles.
	mController->setCubeVelocity();
	double best_ctrl = MyControlPBP();
	std::cout<<"Best CTRL: "<<best_ctrl<<std::endl;
	mController->setCubeAcc(best_ctrl);

	SimWindow::timeStepping();
}

bool MyWindow::simCube(float *state, float ctrlAcc, float *nextState, double &pos_dof0, double &pos_dof2, double &vel_dof2, WorldPtr mSubWorld, Controller* mSubController, int smpl_idx, int tim_idx)
{
	bool collision = false;
	mSubWorld->getSkeleton("cube")->getDof(0)->setPosition(pos_dof0);
	mSubWorld->getSkeleton("cube")->getDof(1)->setPosition(state[0]);
	mSubWorld->getSkeleton("cube")->getDof(2)->setPosition(pos_dof2);
	mSubWorld->getSkeleton("cube")->getDof(0)->setVelocity(mSubController->mSpeed);
	mSubWorld->getSkeleton("cube")->getDof(1)->setVelocity(state[1]);
	mSubWorld->getSkeleton("cube")->getDof(2)->setVelocity(vel_dof2);
	
	std::cout<<"---------------------------------------------------------------"<<std::endl;
	std::cout<<"at "<<tim_idx<<" rendering "<<smpl_idx<<"th sample, "<<"ctrl is "<<ctrlAcc<<std::endl;
	std::cout<<pos_dof0<<" "<<state[0]<<" "<<pos_dof2<<" "<<state[1]<<" "<<vel_dof2<<std::endl;
	render();
	glFlush();

	if (mSubController->collisionEvent())
	{
		collision = true;
		nextState[0] = state[0];
		nextState[1] = state[1];
	}
	else
	{
		mSubController->setCubeAcc(ctrlAcc);	
		mSubWorld->step();
		nextState[0] = mSubWorld->getSkeleton("cube")->getDof(1)->getPosition(); 
		nextState[1] = mSubWorld->getSkeleton("cube")->getDof(1)->getVelocity(); 
		pos_dof0     = mSubWorld->getSkeleton("cube")->getDof(0)->getPosition();
		pos_dof2     = mSubWorld->getSkeleton("cube")->getDof(2)->getPosition();
		vel_dof2     = mSubWorld->getSkeleton("cube")->getDof(2)->getVelocity();
		if (mSubController->collisionEvent())
		{
			collision = true;
		}
	}
	
	std::cout<<"___________After world simulate one step further______________"<<std::endl;
	std::cout<<pos_dof0<<" "<<nextState[0]<<" "<<pos_dof2<<" "<<nextState[1]<<" "<<vel_dof2<<std::endl;
	std::cout<<"---------------------------------------------------------------"<<std::endl<<std::endl;
	return collision;
}

double MyWindow::MyControlPBP()
{
	// parameters need to be tuned
	// nSamples
	// nTimeSteps
	// controlMean
	// C
	// controlStd
	// controlDiffStd
	// controlDiffDiffStd
	// stateStd  -- can be NULL 1. set as NULL
	// mutationScale
	// pbp.Params  1. No backward pass
	// cost

	// clone a world
	WorldPtr mSubWorld			    = std::make_shared<World>();
	mSubWorld						= mWorld->clone();
	mSubWorld->setTimeStep(mNewTimeStep);
	double position_record_dof_0    = mWorld->getSkeleton("cube")->getDof(0)->getPosition();
	double position_record_dof_1    = mWorld->getSkeleton("cube")->getDof(1)->getPosition();
	double position_record_dof_2    = mWorld->getSkeleton("cube")->getDof(2)->getPosition();
	double velocity_record_dof_0    = mWorld->getSkeleton("cube")->getDof(0)->getVelocity();
	double velocity_record_dof_1    = mWorld->getSkeleton("cube")->getDof(1)->getVelocity();
	double velocity_record_dof_2    = mWorld->getSkeleton("cube")->getDof(2)->getVelocity();

	WorldPtr tmp = mWorld;
	setWorld(mSubWorld);

	Controller*	mSubController;
	dart::collision::CollisionDetector* mSubDetector;

	mSubWorld->getConstraintSolver()->setCollisionDetector(new dart::collision::BulletCollisionDetector());
	mSubDetector = mSubWorld->getConstraintSolver()->getCollisionDetector();
	mSubDetector->detectCollision(true, true);
	mSubController = new Controller(mSubWorld->getSkeleton("cube"),mSubWorld->getSkeleton("world_setup"), mSubDetector);

	//initialize the optimizer
	AaltoGames::ControlPBP pbp;
	const int nSamples				= 3;	//N in the paper
	int nTimeSteps				    = 300;	//K in the paper
	const int nStateDimensions		= 2;
	const int nControlDimensions	= 1;
	float minControl				= -mSubController->mAcc;	//lower sampling bound
	float maxControl				= mSubController->mAcc;	//upper sampling bound
	float controlMean=0;	//we're using torque as the control, makes sense to have zero mean
	//Square root of the diagonal elements of C_u in the paper, i.e., stdev of a Gaussian prior for control.
	//Note that the optimizer interface does not have the C_u as a parameter, and instead uses meand and stdev arrays as parameters. 
	float C							= 10;	
	float controlStd				= 1.0f*C;	//sqrt(\sigma_{0}^2 C_u) of the paper (we're not explicitly specifying C_u as u is a scalar here). In effect, a "tolerance" for torque minimization in this test
	float controlDiffStd			= 1.0f*C;	//sqrt(\sigma_{1}^2 C_u) in the paper. In effect, a "tolerance" for angular jerk minimization in this test
	float controlDiffDiffStd		= 100.0f*C; //sqrt(\sigma_{2}^2 C_u) in the paper. A large value to have no effect in this test.
	//float stateStd[nStateDimensions]= {1e-3, 1e-3};	//square roots of the diagonal elements of Q in the paper
	float* stateStd = NULL;
	float mutationScale=0.25f;		//\sigma_m in the paper
	pbp.init(nSamples,nTimeSteps,nStateDimensions,nControlDimensions,&minControl,&maxControl,&controlMean,&controlStd,&controlDiffStd,&controlDiffDiffStd,mutationScale,stateStd);

	//set further params: portion of "no prior" samples, resampling threshold, whether to use the backwards smoothing pass, and the regularization of the smoothing pass
	pbp.setParams(0.1f,0.5f,false,0.001f);  

	//allocate simulation states
	float state[nSamples][nStateDimensions];
	float nextState[nSamples][nStateDimensions];


	float masterState[nStateDimensions];
	masterState[0] = position_record_dof_1; 
	masterState[1] = velocity_record_dof_1; 

	//signal the start of new C-PBP iteration
	pbp.startIteration(true,masterState);
	//init all random walker states to the master state
	for (int i=0; i<nSamples; i++)
	{
		std::memcpy(&state[i],masterState,sizeof(float)*nStateDimensions);
	}

	// keeping the horizontal position for each samples
	double pos_dof0[nSamples];
	for (int i=0; i<nSamples; i++)
	{
		pos_dof0[i] = position_record_dof_0;
	}
	// keeping the z position for each samples
	double pos_dof2[nSamples];
	for (int i=0; i<nSamples; i++)
	{
		pos_dof2[i] = position_record_dof_2;
	}
	// keeping the z velocity for each samples
	double vel_dof2[nSamples];
	for (int i=0; i<nSamples; i++)
	{
		vel_dof2[i] = velocity_record_dof_2;
	}

	//simulate forward 
	for (int k=0; k<nTimeSteps; k++)
	{
		//signal the start of a planning step
		pbp.startPlanningStep(k);
		//NOTE: for multithreaded operation, one would typically run each iteration of the following loop in a separate thread. 
		//The getControl(), getPreviousSampleIdx(), and updateResults() methods of the optimizer are thread-safe. Regarding the physics simulation,
		//one would typically have a separate instance of the whole physics world for each thread, with full physics state stored/loaded similar to the state and nextState arrays here.
		
		//#pragma omp parallel for num_threads(8)	
		for (int i=0; i<nSamples; i++)
		{
			//get control from C-PBP
			float control;
			pbp.getControl(i,&control);

			//get the mapping from this to previous state (affected by resampling operations)
			int previousStateIdx=pbp.getPreviousSampleIdx(i);

			// set openmp lock
			//omp_set_lock(&lock);
			//simulate to get next state.
			// bool collision_checking = simCube(state[previousStateIdx],control,nextState[i],pos_dof0[i], pos_dof2[i], vel_dof2[i], mSubWorld, mSubController, i, k);
			bool collision_checking = simCube(state[i],control,nextState[i],pos_dof0[i], pos_dof2[i], vel_dof2[i], mSubWorld, mSubController, i, k);
			// unzet openmp lock
			//omp_unset_lock(&lock);

			//evaluate state cost
			float cost=AaltoGames::squared(nextState[i][0]  *10.0f ) + AaltoGames::squared(control  *10.0f ) + AaltoGames::squared((0.3625 - pos_dof0[i]) *5.0f)+ float(collision_checking) * 100.0f;

			//store the state and cost to C-PBP. Note that in general, the stored state does not need to contain full simulation state as in this simple case.
			//instead, one may use arbitrary state features
			pbp.updateResults(i,&control,nextState[i],cost);
		}

		// testify what the heck those bookkeeping variables are
		for (int i=0; i<nSamples; i++)
		{
			std::cout<<std::endl;
			std::cout<<"***************************************************************";
			std::cout<<std::endl;
			std::cout<<"        "<<i<<"th sample's bookkeeping parameters"<<std::endl;
			std::cout<<"pos_dof0  ---  "<<pos_dof0[i]<<std::endl;
			std::cout<<"pos_dof2  ---  "<<pos_dof2[i]<<std::endl;
			std::cout<<"vel_dof2  ---  "<<vel_dof2[i]<<std::endl;
			std::cout<<"***************************************************************";
			std::cout<<std::endl;
		}

		//update all states, will be used at the next step
		std::memcpy(state,nextState,sizeof(state));

		//signal the end of the planning step. this normalizes the state costs etc. for the next step
		pbp.endPlanningStep(k);

	}
	//signal the end of an iteration. this also executes the backwards smoothing pass
	pbp.endIteration();

//		Eigen::MatrixXf tmpSC = pbp.stateAndControlToMatrix();
//		printStatsToFile(fileForStatesAndControls,tmpSC);

	//deploy the best control found
	float control;
	pbp.getBestControl(0,&control);

	setWorld(tmp);
	return control;
}


} // namespace toyexample
