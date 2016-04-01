/*************************************************************************
    > File Name: MyWindow.cpp > Author: Yang Tian
    > Mail: tianyang9310@gmail.com 
    > Created Time: Wed 16 Mar 2016 11:53:26 AM EDT
 ************************************************************************/

#include "MyWindow.h"
#include <time.h>

#define PRINT_CONTROL_PBP false

namespace toyexample{


MyWindow::MyWindow(WorldPtr world):N(32),K(300),traj_dof0_x(N,K),traj_dof1_y(N,K)
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

	traj_dof0_x.setZero();
	traj_dof1_y.setZero();

	targetPos_dof0_x       = 0.3625;
	targetPos_dof1_y       = 0.0;
	targetVel_dof0_x       = 0.0;
	targetVel_dof1_y       = 0.0;
	delta_targetPos_dof1_y = 0.003;

	obstacle_idx		   = 0;
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

void MyWindow::drawSkels()
{
	//mWorld->getSkeleton("cube")->getBodyNode(0)->getParentJoint()->getTransformFromParentBodyNode().translation().x()	
	// is constantly -0.175
	//mWorld->getSkeleton("cube")->getBodyNode(0)->getTransform().translation().matrix().block<1,1>(0, 0)
	// is increased from -0.175
	//mWorld->getSkeleton("cube")->getDof(0)->getPosition()
	// is increased from 0

	// ----------------------------------------------------------------------------------------------
	//					 draw trajectories according to traj_dof0_x and traj_dof1_y
	if ( (!traj_dof0_x.isZero()) || (!traj_dof1_y.isZero()) )
    {
		double mLineWidth = 1.0;
		glLineWidth(mLineWidth);
		glColor3f(0.42,0.42,0.42);
		for (int i=0; i<N; i++)
		{
			glBegin(GL_LINE_STRIP);
			for (int j=0; j<K; j++)
			{
				glVertex2f(traj_dof0_x(i,j) + mWorld->getSkeleton("cube")->getBodyNode(0)->getParentJoint()->getTransformFromParentBodyNode().translation().x(), 
						   traj_dof1_y(i,j) + mWorld->getSkeleton("cube")->getBodyNode(0)->getParentJoint()->getTransformFromParentBodyNode().translation().y());
			}
			glEnd();
		}
	}
	// ----------------------------------------------------------------------------------------------
	

	// ----------------------------------------------------------------------------------------------
	//								draw traget position
	float target_radius = 0.005;
	glBegin(GL_TRIANGLE_FAN);
	glColor3f(0,1,0);
	for (float i = 90; i <270; i=i+0.5)
	{
		glVertex2f((targetPos_dof0_x + mWorld->getSkeleton("cube")->getBodyNode(0)->getParentJoint()->getTransformFromParentBodyNode().translation().x() + target_radius * cos(i*DART_RADIAN)),
				   (targetPos_dof1_y + target_radius * sin(i*DART_RADIAN)));
	}
	glEnd();
	// ----------------------------------------------------------------------------------------------


	SimWindow::drawSkels();
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

	if (PRINT_CONTROL_PBP)
	{
		std::cout<<"---------------------------------------------------------------"<<std::endl;
		std::cout<<"at "<<tim_idx<<" rendering "<<smpl_idx<<"th sample, "<<"ctrl is "<<ctrlAcc<<std::endl;
		std::cout<<pos_dof0<<" "<<state[0]<<" "<<pos_dof2<<" "<<state[1]<<" "<<vel_dof2<<std::endl;
	}
	//render();
	//glFlush();

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
	
	if (PRINT_CONTROL_PBP)
	{
		std::cout<<"___________After world simulate one step further______________"<<std::endl;
		std::cout<<pos_dof0<<" "<<nextState[0]<<" "<<pos_dof2<<" "<<nextState[1]<<" "<<vel_dof2<<std::endl;
		std::cout<<"---------------------------------------------------------------"<<std::endl<<std::endl;
	}
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
	
	// setting of nSamples and nTimeSteps, go to constructor of MyWindow, and set N and K
	const int nSamples				= N;	//N in the paper
    int nTimeSteps					= K;	//K in the paper
	const int nStateDimensions		= 2;
	const int nControlDimensions	= 1;
	float minControl				= -mSubController->mAcc;	//lower sampling bound
	float maxControl				= mSubController->mAcc;	//upper sampling bound
	float controlMean				= 0.0f;	//we're using torque as the control, makes sense to have zero mean
	//Square root of the diagonal elements of C_u in the paper, i.e., stdev of a Gaussian prior for control.
	//Note that the optimizer interface does not have the C_u as a parameter, and instead uses meand and stdev arrays as parameters. 
	float C							= maxControl * 2.0;	
	float controlStd				= 1.0f*C;	//sqrt(\sigma_{0}^2 C_u) of the paper (we're not explicitly specifying C_u as u is a scalar here). In effect, a "tolerance" for torque minimization in this test
	float controlDiffStd			= 100.0f*C;	//sqrt(\sigma_{1}^2 C_u) in the paper. In effect, a "tolerance" for angular jerk minimization in this test
	float controlDiffDiffStd		= 100.0f*C; //sqrt(\sigma_{2}^2 C_u) in the paper. A large value to have no effect in this test.
	//float stateStd[nStateDimensions]= {1e-3, 1e-3};	//square roots of the diagonal elements of Q in the paper
	float* stateStd					= NULL;
	float mutationScale				=0.1f;		//\sigma_m in the paper, larger sigma_m means that ctrl of last timestep has little impact on current ctrl
	pbp.init(nSamples,nTimeSteps,nStateDimensions,nControlDimensions,&minControl,&maxControl,&controlMean,&controlStd,&controlDiffStd,&controlDiffDiffStd,mutationScale,stateStd);

	//set further params: portion of "no prior" samples, resampling threshold, whether to use the backwards smoothing pass, and the regularization of the smoothing pass
	
	// ----------------------------------------------------------------------------------------------
	//           if no resampling, we can directly set resampling threshold as 0 
	//	since if resmpl_thre < 1/N, then no resampling, please note that false here indicates
	//						wether there is backwards smoothing pass
	//
	//pbp.setParams(0.1f,0.5f,false,0.001f);  
	pbp.setParams(0.1f,0.0f,false,0.001f);  
	// ----------------------------------------------------------------------------------------------

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


			// set openmp lock
			//omp_set_lock(&lock);
			//simulate to get next state.
			
			// --------------------------------------------------------------------------------------------------------------------
			//          we disable resampling precedure in setParams, actually previousStateIdx can always return the 
			//          correct index. In other words, if resampling, then previousStateIdx can link to the correct
			//          previous state; if not resampling, previousStateIdx is simply i
			//
			// get the mapping from this to previous state (affected by resampling operations)
			int previousStateIdx=pbp.getPreviousSampleIdx(i);
			bool collision_checking = simCube(state[previousStateIdx],control,nextState[i],pos_dof0[i], pos_dof2[i], vel_dof2[i], mSubWorld, mSubController, i, k);
			// --------------------------------------------------------------------------------------------------------------------
			
			// unzet openmp lock
			//omp_unset_lock(&lock);

			// keep record of x (dof0) and y (dof1) of points in trajectory
			traj_dof0_x(i,k) = pos_dof0[i];
			traj_dof1_y(i,k) = nextState[i][0];

			// --------------------------------------------------------------------------------------------------------------------
			//											  evaluate state cost
			float cost			   = AaltoGames::squared((pos_dof0[i]     - targetPos_dof0_x) *10.0f) +  // horizontal distance between cube and target
									 AaltoGames::squared((nextState[i][0] - targetPos_dof1_y) *10.0f) +  // vertical distance between cube and target
									 AaltoGames::squared((nextState[i][1] - targetVel_dof1_y) *10.0f) +  // vertical velocity of cube
									 //AaltoGames::squared(control							  *10.0f) +  // control
									 float(collision_checking) * 100.0f;
			// --------------------------------------------------------------------------------------------------------------------

			//store the state and cost to C-PBP. Note that in general, the stored state does not need to contain full simulation state as in this simple case.
			//instead, one may use arbitrary state features
			pbp.updateResults(i,&control,nextState[i],cost);
		}

		// testify what the heck those bookkeeping variables are
		if (PRINT_CONTROL_PBP)
		{
			for (int i=0; i<nSamples; i++)
			{
				std::cout<<std::endl;
				std::cout<<"***************************************************************";
				std::cout<<std::endl;
				std::cout<<"        "<<i<<"th sample's bookkeeping parameters"<<std::endl;
				std::cout<<"		pos_dof0  ---  "<<pos_dof0[i]<<std::endl;
				std::cout<<"		pos_dof2  ---  "<<pos_dof2[i]<<std::endl;
				std::cout<<"		vel_dof2  ---  "<<vel_dof2[i]<<std::endl;
				std::cout<<"***************************************************************";
				std::cout<<std::endl;
			}
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

	render();
	glFlush();

	return control;
}

void MyWindow::keyboard(unsigned char key, int x, int y)
{
	switch(key)
	{
		case '1':
		// move target 
		obstacle_idx      = 0;
		break;

		case '2':
		// move first obstacle
		obstacle_idx      = 2;
		break;

		case '3':
		// move second obstacle
		obstacle_idx      = 1;
		break;

		case '4':
		// move third obstacle
		obstacle_idx      = 3;
		break;

		case 'h':
		// move left
		moveObject(-delta_targetPos_dof1_y, false);
		break;

		case 'j':
		// move down
		moveObject(-delta_targetPos_dof1_y, true);
		break;

		case 'k':
		// move up
		moveObject(delta_targetPos_dof1_y, true);
		break;

		case 'l':
		// move right
		moveObject(delta_targetPos_dof1_y, false);
		break;

		case 'r':
		// reset
		resetObject();
		break;

		default:
		SimWindow::keyboard(key, x, y); // ' ', 'p', '[', ']', 'v', 's', ',', '.', 'c', 'ESC'
	}
}

void MyWindow::moveObject(float delta, bool Y_direction)
{
	if (obstacle_idx == 0)
	{
		if (Y_direction)
		{
			targetPos_dof1_y     +=delta;
		}
	}
	else
	{
		BodyNodePtr obstacle  = mWorld->getSkeleton("world_setup")->getBodyNode("obstacle_"+std::to_string(obstacle_idx));
		Eigen::Isometry3d tf  = obstacle->getParentJoint()->getTransformFromParentBodyNode();
		if (Y_direction)
		{
			tf.translation().y() += delta;
		}
		else
		{
			tf.translation().x() += delta;
		}
		obstacle->getParentJoint()->setTransformFromParentBodyNode(tf);
	}
}

 
void MyWindow::resetObject()
{
	extern const double floor_length;
	extern const double obstacle_radius;
	extern const double obstacle_2_wall;
	extern const double obstacle_height;
	targetPos_dof1_y = 0;
	for (int i = 1; i<4; i++)
	{
		BodyNodePtr obstacle  = mWorld->getSkeleton("world_setup")->getBodyNode("obstacle_"+std::to_string(i));
		Eigen::Isometry3d tf  = obstacle->getParentJoint()->getTransformFromParentBodyNode();
		if (i < 2)
		{
			tf.translation() = Eigen::Vector3d(0.0, -floor_length/4 + obstacle_radius +  obstacle_2_wall, obstacle_height/2.0);
		}
		else
		{
			tf.translation() = Eigen::Vector3d(((i - 2.5)*2)*floor_length/4.0, floor_length/4 - obstacle_radius/2.0 - obstacle_2_wall , obstacle_height/2.0);
		}
		obstacle->getParentJoint()->setTransformFromParentBodyNode(tf);
	}
}


} // namespace toyexample
