/*************************************************************************
    > File Name: toy_example_MPC.cpp
    > Author: Yang Tian
    > Mail: tianyang9310@gmail.com 
    > Created Time: Sun Feb  7 21:14:13 2016
 ************************************************************************/
#include "dart/dart.h"
#include <iostream>
#include <time.h>
#include <vector>
#include <algorithm>
#include <cstdlib>
#include "ControlPBP/ControlPBP.h"
#include "ControlPBP/Eigen/Eigen"

using namespace dart::common;
using namespace dart::utils;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;
using namespace dart::gui;

const double transparency = 0.3;

const double floor_length = 0.4;
const double floor_height = 0.01;
const double wall_height = floor_length / 8.0;
const double wall_thickness = floor_height;
const double wall_length = floor_length;
const double obstacle_radius = 0.05;
const double obstacle_height = wall_height / 2.0;

const double cube_length = 0.005;

const double obstacle_2_wall = 2.5*wall_thickness;

const double default_density = 2e3;

#define Horizontal_V 1

#define Vertical_A 5000

class Controller
{
public:
	Controller(const SkeletonPtr& cube, dart::collision::CollisionDetector* detector)
		:mCube(cube),mDetector(detector)
	{
		gain = 1;

		mSpeed = Horizontal_V; 

		mAcceleration_random = Vertical_A;

		mCube->getJoint(0)->setActuatorType(Joint::FORCE);

		std::srand((unsigned int)time(NULL));
	}

	bool collision_with_obstacles()
	{
		bool collision = false;
		size_t collision_count = mDetector->getNumContacts();
		for (size_t i = 0; i< collision_count; ++i)
		{
			const dart::collision::Contact& contact = mDetector->getContact(i);

			if ((contact.bodyNode1.lock()->getName() == "obstacle1" && 
						contact.bodyNode2.lock()->getSkeleton()->getName() == "cube")|| 
				(contact.bodyNode2.lock()->getName() == "obstacle1" && 
						contact.bodyNode1.lock()->getSkeleton()->getName() == "cube")|| 
				(contact.bodyNode1.lock()->getName() == "obstacle2" && 
						contact.bodyNode2.lock()->getSkeleton()->getName() == "cube")|| 
				(contact.bodyNode2.lock()->getName() == "obstacle2" && 
						contact.bodyNode1.lock()->getSkeleton()->getName() == "cube")|| 
				(contact.bodyNode1.lock()->getName() == "obstacle3" && 
						contact.bodyNode2.lock()->getSkeleton()->getName() == "cube")|| 
				(contact.bodyNode2.lock()->getName() == "obstacle3" && 
						contact.bodyNode1.lock()->getSkeleton()->getName() == "cube")||
				(contact.bodyNode1.lock()->getName() == "wall3" && 
						contact.bodyNode2.lock()->getSkeleton()->getName() == "cube")|| 
				(contact.bodyNode2.lock()->getName() == "wall3" && 
						contact.bodyNode1.lock()->getSkeleton()->getName() == "cube")||
				(contact.bodyNode1.lock()->getName() == "wall4" && 
						contact.bodyNode2.lock()->getSkeleton()->getName() == "cube")|| 
				(contact.bodyNode2.lock()->getName() == "wall4" && 
						contact.bodyNode1.lock()->getSkeleton()->getName() == "cube"))
			/*if (contact.bodyNode1.lock()->getSkeleton()->getName() == "cube" ||
					contact.bodyNode2.lock()->getSkeleton()->getName() == "cube") */
			{
				collision = true;
				//std::cout<<"collision with OBSTACLES detected!"<<std::endl;
				break;
			}
		}
		return collision;
	}
	void setCubeVelocity(double speed)
	{
		//if (!collision_with_obstacles())
		//{
			mCube->getDof(0)->setVelocity(speed);
		//}
	}

	double setCubeAcceleration()
	{
		// increase the magnitude of acc randomization
		if (collision_with_obstacles())
		{
			std::cout<<"increase magnitude of acc randomization"<<std::endl;
			mAcceleration_random = mAcceleration_random * 1.01;
		}
		else
		{
			std::cout<<"restore magnitude of acc randomization"<<std::endl;
			mAcceleration_random = Vertical_A;
		}
		randomizeAcceleration();
		mCube->getDof(1)->setCommand(mCube->getMass() * mAcceleration);
		return mAcceleration;	
	}


	void setCubeAcceleration(double desire_Acceleration)
	{
		mCube->getDof(1)->setCommand(mCube->getMass() * desire_Acceleration);
	}

	void randomizeAcceleration()
	{
		mAcceleration = mAcceleration_random * (std::rand() / double(RAND_MAX) - 0.5)*2;
	}

	friend class MyWindow;

protected:
	SkeletonPtr mCube;
	double mAcceleration;
	double mSpeed;

	double mAcceleration_random;
	
	dart::collision::CollisionDetector* mDetector;

	double gain;
};

class MyWindow : public dart::gui::SimWindow
{
public:
	MyWindow(const WorldPtr& world)
	{
		setWorld(world);

		mWorld->getConstraintSolver()->setCollisionDetector(new dart::collision::BulletCollisionDetector());
		detector = mWorld->getConstraintSolver()->getCollisionDetector();
		detector->detectCollision(true, true);

		mController = std::unique_ptr<Controller>(new Controller(mWorld->getSkeleton("cube"),detector));
	}

	int simulateCube(float *state, float controlAcceleration, float *nextState)
	{
		int collision = 0;
		mWorld->getSkeleton("cube")->getDof(0)->setPosition(state[0]);
		mWorld->getSkeleton("cube")->getDof(1)->setPosition(state[1]);
		mWorld->getSkeleton("cube")->getDof(2)->setPosition(0);	
		// don't need to set velocity of dof 0
		mWorld->getSkeleton("cube")->getDof(1)->setVelocity(state[2]);	
		mWorld->getSkeleton("cube")->getDof(2)->setVelocity(0);	
		if (mController->collision_with_obstacles())
		{
			collision = 1;
			nextState[0] =state[0] ; 
			nextState[1] =state[1] ; 
			nextState[2] =state[2] ; 
			return collision;
		}
		mController->setCubeVelocity( mController->gain * mController->mSpeed);
		mController->setCubeAcceleration(controlAcceleration);
		mWorld->step();

		nextState[0] = mWorld->getSkeleton("cube")->getDof(0)->getPosition(); 
		nextState[1] = mWorld->getSkeleton("cube")->getDof(1)->getPosition(); 
		nextState[2] = mWorld->getSkeleton("cube")->getDof(1)->getVelocity(); 
		if (mController->collision_with_obstacles())
		{
			collision = 1;
		}
		return collision;
	}

	double MyControlPBP()
	{
		//srand ((unsigned int)time(NULL));

		//initialize the optimizer
		AaltoGames::ControlPBP pbp;
		const int nSamples = 128;	//N in the paper
		int nTimeSteps     = 400;		//K in the paper, resulting in a 0.5s planning horizon
		//const float PI=3.1416f;	
		const int nStateDimensions=3;
		const int nControlDimensions=1;
		float minControl=-mController->mAcceleration_random;	//lower sampling bound
		float maxControl=mController->mAcceleration_random;		//upper sampling bound
		float controlMean=0;	//we're using torque as the control, makes sense to have zero mean
		//Square root of the diagonal elements of C_u in the paper, i.e., stdev of a Gaussian prior for control.
		//Note that the optimizer interface does not have the C_u as a parameter, and instead uses meand and stdev arrays as parameters. 
		//The 3D character tests compute the C_u on the Unity side to reduce the number of effective parameters, and then compute the arrays based on it as described to correspond to the products \sigma_0 C_u etc.
		float C=mController->mAcceleration_random;	
		float controlStd=1.0f*C;	//sqrt(\sigma_{0}^2 C_u) of the paper (we're not explicitly specifying C_u as u is a scalar here). In effect, a "tolerance" for torque minimization in this test
		float controlDiffStd=1.0f*C;	//sqrt(\sigma_{1}^2 C_u) in the paper. In effect, a "tolerance" for angular jerk minimization in this test
		float controlDiffDiffStd=100.0f*C; //sqrt(\sigma_{2}^2 C_u) in the paper. A large value to have no effect in this test.
		float stateStd[3]={1e-18, 1e-18, 1e-18};	//square roots of the diagonal elements of Q in the paper
		//float* stateStd = NULL;
		float mutationScale=0.25f;		//\sigma_m in the paper
		pbp.init(nSamples,nTimeSteps,nStateDimensions,nControlDimensions,&minControl,&maxControl,&controlMean,&controlStd,&controlDiffStd,&controlDiffDiffStd,mutationScale,stateStd);

		//set further params: portion of "no prior" samples, resampling threshold, whether to use the backwards smoothing pass, and the regularization of the smoothing pass
		pbp.setParams(0.1f,0.5f,true,0.001f);  

		//allocate simulation states
		float state[nSamples][nStateDimensions];
		float nextState[nSamples][nStateDimensions];

		double position_record_dof_0 = mWorld->getSkeleton("cube")->getDof(0)->getPosition();
		double position_record_dof_1 = mWorld->getSkeleton("cube")->getDof(1)->getPosition();
		double position_record_dof_2 = mWorld->getSkeleton("cube")->getDof(2)->getPosition();
		double velocity_record_dof_0 = mWorld->getSkeleton("cube")->getDof(0)->getVelocity();
		double velocity_record_dof_1 = mWorld->getSkeleton("cube")->getDof(1)->getVelocity();
		double velocity_record_dof_2 = mWorld->getSkeleton("cube")->getDof(2)->getVelocity();

		float masterState[3];
		masterState[0] = position_record_dof_0; 
		masterState[1] = position_record_dof_1; 
		masterState[2] = velocity_record_dof_1; 

		//signal the start of new C-PBP iteration
		pbp.startIteration(true,masterState);
		//init all random walker states to the master state
		for (int i=0; i<nSamples; i++)
		{
			std::memcpy(&state[i],masterState,sizeof(float)*nStateDimensions);
		}

		//simulate forward 
		for (int k=0; k<nTimeSteps; k++)
		{
			//signal the start of a planning step
			pbp.startPlanningStep(k);
			//NOTE: for multithreaded operation, one would typically run each iteration of the following loop in a separate thread. 
			//The getControl(), getPreviousSampleIdx(), and updateResults() methods of the optimizer are thread-safe. Regarding the physics simulation,
			//one would typically have a separate instance of the whole physics world for each thread, with full physics state stored/loaded similar to the state and nextState arrays here.
			for (int i=0; i<nSamples; i++)
			{
				//get control from C-PBP
				float control;
				pbp.getControl(i,&control);

				//get the mapping from this to previous state (affected by resampling operations)
				int previousStateIdx=pbp.getPreviousSampleIdx(i);

				//simulate to get next state.
				int collision_checkout = simulateCube(state[previousStateIdx],control,nextState[i]);

				//evaluate state cost
				float cost=AaltoGames::squared(nextState[i][1]  *10.0f ) + AaltoGames::squared(control  *10.0f ) + collision_checkout * 100.0f;

				//store the state and cost to C-PBP. Note that in general, the stored state does not need to contain full simulation state as in this simple case.
				//instead, one may use arbitrary state features
				pbp.updateResults(i,&control,nextState[i],cost);
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


		//output the whole best control on the trajectory
		for (int k=0; k<nTimeSteps; k++)
		{
			float tmp_control;
			pbp.getBestControl(k,&tmp_control);
			std::cout<<tmp_control<<std::endl;
		}

		mWorld->getSkeleton("cube")->getDof(0)->setPosition(position_record_dof_0);
		mWorld->getSkeleton("cube")->getDof(1)->setPosition(position_record_dof_1);
		mWorld->getSkeleton("cube")->getDof(2)->setPosition(0);	
		mWorld->getSkeleton("cube")->getDof(0)->setVelocity(mController->mSpeed);	
		mWorld->getSkeleton("cube")->getDof(1)->setVelocity(velocity_record_dof_1);	
		mWorld->getSkeleton("cube")->getDof(2)->setVelocity(0);	

		return control;
	}

	double MyMPC()
	{
		int num_samples = 20;
		int plan_horizon = 100;
		double desire_Acceleration = 0;
		
		double position_record_dof_0 = mWorld->getSkeleton("cube")->getDof(0)->getPosition();
		double position_record_dof_1 = mWorld->getSkeleton("cube")->getDof(1)->getPosition();
		double position_record_dof_2 = mWorld->getSkeleton("cube")->getDof(2)->getPosition();
		double velocity_record_dof_0 = mWorld->getSkeleton("cube")->getDof(0)->getVelocity();
		double velocity_record_dof_1 = mWorld->getSkeleton("cube")->getDof(1)->getVelocity();
		double velocity_record_dof_2 = mWorld->getSkeleton("cube")->getDof(2)->getVelocity();

		std::vector<double> acceleration_array(num_samples);
		std::vector<double> cost_array(num_samples);
	
		for (int i = 0; i<num_samples;i++)
		{
			cost_array[i] = plan_horizon * mWorld->getTimeStep();
		}

		for (int i = 0; i<num_samples; i++)
		{
			double time_start = mWorld->getTime();

			mWorld->getSkeleton("cube")->getDof(0)->setPosition(position_record_dof_0);
			mWorld->getSkeleton("cube")->getDof(1)->setPosition(position_record_dof_1);
			mWorld->getSkeleton("cube")->getDof(2)->setPosition(position_record_dof_2);	
			mWorld->getSkeleton("cube")->getDof(0)->setVelocity(velocity_record_dof_0);	
			mWorld->getSkeleton("cube")->getDof(1)->setVelocity(velocity_record_dof_1);	
			mWorld->getSkeleton("cube")->getDof(2)->setVelocity(velocity_record_dof_2);	

			mController->setCubeVelocity(mController->gain * mController->mSpeed);
			acceleration_array[i] = mController->setCubeAcceleration();
			for (int j = 0; j<plan_horizon; j++)
			{
				mWorld->step();
				// whether need to maginify the horizontal velocity
				mController->setCubeVelocity( mController->gain * mController->mSpeed);
				mController->setCubeAcceleration();
				
				if (mController->collision_with_obstacles())
				{
					//std::cout<<"collision with OBSTACLES detected!"<<std::endl;
					cost_array[i] = mWorld->getTime() - time_start;
					break;
				}
			}
		}

		desire_Acceleration = acceleration_array[std::distance(cost_array.begin(), std::max_element(cost_array.begin(), cost_array.end()))];
		
		// compute forward
		// find the best acceleration
		mWorld->getSkeleton("cube")->getDof(0)->setPosition(position_record_dof_0);
		mWorld->getSkeleton("cube")->getDof(1)->setPosition(position_record_dof_1);
		mWorld->getSkeleton("cube")->getDof(2)->setPosition(position_record_dof_2);	
		mWorld->getSkeleton("cube")->getDof(0)->setVelocity(velocity_record_dof_0);	
		mWorld->getSkeleton("cube")->getDof(1)->setVelocity(velocity_record_dof_1);	
		mWorld->getSkeleton("cube")->getDof(2)->setVelocity(velocity_record_dof_2);	
		return desire_Acceleration;
	}

	void timeStepping() override
	{
		if (mController->collision_with_obstacles())
		{
			std::cout<<"collision with obstacles detected!"<<std::endl;
		}

		double best_control_from_controlPBP = MyControlPBP();
		mController->setCubeAcceleration(best_control_from_controlPBP);
		// guarantee the ball always has horizontal velocity
		mController->setCubeVelocity(mController->mSpeed);

		
		std::cout<<"time is "<<mWorld->getTime()<<std::endl;
		std::cout<<"p_x ="<<mWorld->getSkeleton("cube")->getDof(0)->getPosition()<<std::endl;
		std::cout<<"p_y ="<<mWorld->getSkeleton("cube")->getDof(1)->getPosition()<<std::endl;
		std::cout<<"p_z ="<<mWorld->getSkeleton("cube")->getDof(2)->getPosition()<<std::endl;
		std::cout<<"v_x ="<<mWorld->getSkeleton("cube")->getDof(0)->getVelocity()<<std::endl;
		std::cout<<"v_y ="<<mWorld->getSkeleton("cube")->getDof(1)->getVelocity()<<std::endl;
		std::cout<<"v_z ="<<mWorld->getSkeleton("cube")->getDof(2)->getVelocity()<<std::endl;
		std::cout<<"best control is "<<best_control_from_controlPBP<<std::endl;


		SimWindow::timeStepping();
		mWorld->getSkeleton("cube")->getDof(2)->setPosition(0);
		mWorld->getSkeleton("cube")->getDof(2)->setVelocity(0);
	}

	void displayTimer(int _val) override
	{
		if (mPlay)
		{
			mPlayFrame +=1;
		}
		else if (mSimulating)
		{
			timeStepping();
			mWorld->bake();
		}
		glutPostRedisplay();
		glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
	}

	void drawSkels() override
	{
		glEnable(GL_LIGHTING);

		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

		SimWindow::drawSkels();
	}
protected:
	std::unique_ptr<Controller> mController;

	dart::collision::CollisionDetector* detector;

	size_t default_Num_contact;
};

SkeletonPtr createFloor()
{
	SkeletonPtr environment = Skeleton::create("environment");

	// create a bodynode 
	BodyNodePtr floor = environment->createJointAndBodyNodePair<WeldJoint>().second;

	floor->setName("floor");

	// attach a shape
	std::shared_ptr<BoxShape> box = std::make_shared<BoxShape>(
			Eigen::Vector3d(floor_length, floor_length/2.0, floor_height));
	box->setColor(dart::Color::Gray(0.3));
	floor->addVisualizationShape(box);
	floor->addCollisionShape(box);

	// set inertia
	dart::dynamics::Inertia inertia;
	inertia.setMass(default_density * box->getVolume());
	inertia.setMoment(box->computeInertia(inertia.getMass()));
	floor->setInertia(inertia);

	// put the body into the right position
	Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
	tf.translation() = Eigen::Vector3d(0.0, 0.0, -floor_height / 2.0);
	floor->getParentJoint()->setTransformFromParentBodyNode(tf);

	return environment;
}

BodyNodePtr addWall(const SkeletonPtr& environment, BodyNodePtr parent, int wall_index)
{
	WeldJoint::Properties properties;
	properties.mName = "wall_weld_joint" + std::to_string(wall_index);

	int flag1, flag2;
	flag1 = 2* ((wall_index - 1) / 2) -1;
	flag2 = 2* ((wall_index - 1 ) % 2) -1;

	// create a bodynode
	BodyNodePtr wall = environment->createJointAndBodyNodePair<WeldJoint>(parent, properties, BodyNode::Properties("wall"+std::to_string(wall_index))).second;

	// attach a shape
	if( flag1 == -1)
	{
		std::shared_ptr<BoxShape> box = std::make_shared<BoxShape>(
			Eigen::Vector3d(wall_thickness, wall_length/2.0, wall_height));
		box->setColor(dart::Color::Gray(transparency));
		wall->addVisualizationShape(box);
		wall->addCollisionShape(box);
		
		// set inertia
		dart::dynamics::Inertia inertia;
		inertia.setMass(default_density * box->getVolume());
		inertia.setMoment(box->computeInertia(inertia.getMass()));
		wall->setInertia(inertia);
	}
	else
	{
		std::shared_ptr<BoxShape> box = std::make_shared<BoxShape>(
			Eigen::Vector3d(wall_length - 2*wall_thickness, wall_thickness, wall_height));
		box->setColor(dart::Color::Fuschia(transparency));
		wall->addVisualizationShape(box);
		wall->addCollisionShape(box);

		// set inertia
		dart::dynamics::Inertia inertia;
		inertia.setMass(default_density * box->getVolume());
		inertia.setMoment(box->computeInertia(inertia.getMass()));
		wall->setInertia(inertia);
	}

	// put the body node into the right position
	Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
	if( flag1 == -1)
	{
		tf.translation() = Eigen::Vector3d(flag2 * (floor_length / 2.0 - wall_thickness /2.0), 0.0, wall_height / 2.0);
	}
	else
	{
		tf.translation() = Eigen::Vector3d(0.0, flag2 * (floor_length / 4.0 -wall_thickness /2.0), wall_height / 2.0);
	}
	wall->getParentJoint()->setTransformFromParentBodyNode(tf);

	return wall;
}

BodyNodePtr addObstacle(const SkeletonPtr& environment, BodyNodePtr parent, int obstacle_index)
{
	WeldJoint::Properties properties;
	properties.mName = "obstacle_weld_joint" + std::to_string(obstacle_index);

	// create a bodynode
	BodyNodePtr obstacle = environment->createJointAndBodyNodePair<WeldJoint>(parent, properties, BodyNode::Properties("obstacle"+std::to_string(obstacle_index))).second;

	// create the shape
	if (obstacle_index <2)
	{
		std::shared_ptr<CylinderShape> cylinder = std::make_shared<CylinderShape>(obstacle_radius, obstacle_height);
		//std::shared_ptr<BoxShape> cylinder = std::make_shared<BoxShape>(Eigen::Vector3d(0.5*obstacle_radius, 2.0*obstacle_radius, obstacle_height));
		cylinder->setColor(dart::Color::Red(transparency));
		obstacle->addVisualizationShape(cylinder);
		obstacle->addCollisionShape(cylinder);
		// set inertia
		dart::dynamics::Inertia inertia;
		inertia.setMass(2 * default_density * cylinder->getVolume());
		inertia.setMoment(cylinder->computeInertia(inertia.getMass()));
		obstacle->setInertia(inertia);
	}
	else
	{
		std::shared_ptr<CylinderShape> cylinder = std::make_shared<CylinderShape>(obstacle_radius/2.0, obstacle_height);
		//std::shared_ptr<BoxShape> cylinder = std::make_shared<BoxShape>(Eigen::Vector3d(0.5*obstacle_radius, 2*obstacle_radius, obstacle_height));
		cylinder->setColor(dart::Color::Red(transparency));
		obstacle->addVisualizationShape(cylinder);
		obstacle->addCollisionShape(cylinder);
		// set inertia
		dart::dynamics::Inertia inertia;
		inertia.setMass(2 * default_density * cylinder->getVolume());
		inertia.setMoment(cylinder->computeInertia(inertia.getMass()));
		obstacle->setInertia(inertia);
	}

	// put it in the position
	Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
	if (obstacle_index < 2)
	{
		tf.translation() = Eigen::Vector3d(0.0, -floor_length/4 + obstacle_radius +  obstacle_2_wall, obstacle_height/2.0);
	}
	else
	{
		tf.translation() = Eigen::Vector3d(((obstacle_index - 2.5)*2)*floor_length/4.0, floor_length/4 - obstacle_radius/2.0 - obstacle_2_wall , obstacle_height/2.0);
	}

	obstacle->getParentJoint()->setTransformFromParentBodyNode(tf);

	return obstacle;
}

SkeletonPtr createCube()
{
	// create a Skeleton
	SkeletonPtr cube = Skeleton::create("cube");

	// create a BodyNode
	BodyNodePtr body = cube->createJointAndBodyNodePair<PlanarJoint>().second;
	
	// create a shape
	std::shared_ptr<BoxShape> ball = std::make_shared<BoxShape>(Eigen::Vector3d(cube_length, cube_length, cube_length));
	//std::shared_ptr<EllipsoidShape> ball = std::make_shared<EllipsoidShape>(Eigen::Vector3d(cube_length, cube_length, cube_length));
	ball->setColor(dart::Color::Black(2*transparency));
	body->addVisualizationShape(ball);
	body->addCollisionShape(ball);
	

	// set inertia
	dart::dynamics::Inertia inertia;
	inertia.setMass(2 * default_density * ball->getVolume());
	inertia.setMoment(ball->computeInertia(inertia.getMass()));
	body->setInertia(inertia);
	std::cout<<"The mass of the ball is "<<cube->getMass()<<std::endl;

	// put it in the right position
	Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
	tf.translation() = Eigen::Vector3d(-floor_length/2.0 + obstacle_2_wall, 0.0, cube_length/2.0);
	body->getParentJoint()->setTransformFromParentBodyNode(tf);

	// output number of joint and DOF
	std::cout<<"The cube has "<<cube->getNumJoints()<<" joints"<<std::endl;
	std::cout<<"The cube has "<<cube->getNumDofs()<<" Dofs"<<std::endl;

	return cube;
}

int main(int argc, char* argv[])
{
	std::cout<<"This is a toy example for Model Predictive Control"<<std::endl;
	SkeletonPtr environment = createFloor();
	addWall(environment, environment->getBodyNode("floor"),1);
	addWall(environment, environment->getBodyNode("floor"),2);
	addWall(environment, environment->getBodyNode("floor"),3);
	addWall(environment, environment->getBodyNode("floor"),4);
	addObstacle(environment, environment->getBodyNode("floor"),1);
	addObstacle(environment, environment->getBodyNode("floor"),2);
	addObstacle(environment, environment->getBodyNode("floor"),3);
	
	SkeletonPtr cube = createCube();

	WorldPtr world = std::make_shared<World>();
	world->addSkeleton(environment);
	world->addSkeleton(cube);

	MyWindow window(world);
	std::cout<<"The world gravity is "<<std::endl<<world->getGravity()<<std::endl;

	glutInit(&argc, argv);
	window.initWindow(640, 480, "A toy example for Model Predictive Control");
	glutMainLoop();
}
