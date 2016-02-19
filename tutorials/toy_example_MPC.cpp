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
const double obstacle_height = wall_height / 4.0;

const double cube_length = 0.01;

const double obstacle_2_wall = 2*wall_thickness;

const double default_density = 2e3;

class Controller
{
public:
	Controller(const SkeletonPtr& cube, dart::collision::CollisionDetector* detector, size_t default_Num_contact, double time_step)
		:mCube(cube),mDetector(detector),mdefault_Num_contact(default_Num_contact), mTime_step_in_Acc_fun(time_step)
	{
		mAcceleration = 0.05;
		mSpeed = 0.2; 

		mAcceleration_random = 2;

		//mCube->getJoint(0)->setActuatorType(Joint::VELOCITY);

		mVelocity_old_in_set_Acc_fun = 0;
		mVelocity_new_in_set_Acc_fun = 0;

		std::srand((unsigned int)time(NULL));
	}

	bool collision_with_obstacles()
	{
		bool collision = false;
		size_t collision_count = mDetector->getNumContacts();
		for (size_t i = 0; i< collision_count; ++i)
		{
			const dart::collision::Contact& contact = mDetector->getContact(i);

			/*
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
						contact.bodyNode1.lock()->getSkeleton()->getName() == "cube"))
			*/
			if (contact.bodyNode1.lock()->getSkeleton()->getName() == "cube" ||
					contact.bodyNode2.lock()->getSkeleton()->getName() == "cube")
			{
				collision = true;
				break;
			}
		}
		return collision;
	}
	void setCubeVelocity(double speed)
	{
		if (!collision_with_obstacles())
		{
			mCube->getDof(0)->setVelocity(speed);
		}
	}

	double setCubeAcceleration()
	{
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

	double mVelocity_old_in_set_Acc_fun;
	double mVelocity_new_in_set_Acc_fun;
	double mTime_step_in_Acc_fun;

	double mAcceleration_random;
	
	dart::collision::CollisionDetector* mDetector;
	size_t mdefault_Num_contact;
};

class MyWindow : public dart::gui::SimWindow
{
public:
	MyWindow(const WorldPtr& world)
	{
		setWorld(world);

		detector = mWorld->getConstraintSolver()->getCollisionDetector();
		detector->detectCollision(true, true);

		default_Num_contact = detector->getNumContacts();

		std::cout<<"Default number of contacts is "<<default_Num_contact<<std::endl;

		mController = std::unique_ptr<Controller>(new Controller(mWorld->getSkeleton("cube"),detector, default_Num_contact, mWorld->getTimeStep()));
	}

	double MyMPC()
	{
		int num_samples = 100;
		int plan_horizon = 10;
		double desire_Acceleration = 0;
		
		double position_record_dof_0 = mWorld->getSkeleton("cube")->getDof(0)->getPosition();
		double position_record_dof_1 = mWorld->getSkeleton("cube")->getDof(1)->getPosition();
		double position_record_dof_2 = mWorld->getSkeleton("cube")->getDof(2)->getPosition();
		double velocity_record_dof_0 = mWorld->getSkeleton("cube")->getDof(0)->getVelocity();
		double velocity_record_dof_1 = mWorld->getSkeleton("cube")->getDof(1)->getVelocity();
		double velocity_record_dof_2 = mWorld->getSkeleton("cube")->getDof(2)->getVelocity();

		//std::vector<WorldPtr> world_array(num_samples);
		//std::vector<std::unique_ptr<Controller>> controller_array(num_samples);
		std::vector<double> acceleration_array(num_samples);
		std::vector<int> cost_array(num_samples);
	
		for (int i = 0; i<num_samples;i++)
		{
			cost_array[i] = plan_horizon * mWorld->getTimeStep();
		}
		
		// save the world
		// have best_acceleration vector and cost vector
		// cost vector is based on the number of contacts.
		for (int i = 0; i<num_samples; i++)
		{
			// states of the skeleton will not be cloned?
			//world_array[i] = mWorld->clone();
			
			double time_start = mWorld->getTime();
			
			mWorld->getSkeleton("cube")->getDof(0)->setPosition(position_record_dof_0);
			mWorld->getSkeleton("cube")->getDof(1)->setPosition(position_record_dof_1);
			mWorld->getSkeleton("cube")->getDof(2)->setPosition(position_record_dof_2);	
			mWorld->getSkeleton("cube")->getDof(0)->setVelocity(velocity_record_dof_0);	
			mWorld->getSkeleton("cube")->getDof(1)->setVelocity(velocity_record_dof_1);	
			mWorld->getSkeleton("cube")->getDof(2)->setVelocity(velocity_record_dof_2);	

			mController->setCubeVelocity(mController->mSpeed);
			acceleration_array[i] = mController->setCubeAcceleration();
			for (int j = 0; j<plan_horizon; j++)
			{
				mWorld->step();
				mController->setCubeVelocity(mController->mSpeed);
				mController->setCubeAcceleration();
				if (mController->collision_with_obstacles())
				{
					std::cout<<"collision with obstacles detected!"<<std::endl;
					mController->setCubeVelocity(0);
					mWorld->getSkeleton("cube")->getDof(0)->setPosition(position_record_dof_0);
					mWorld->getSkeleton("cube")->getDof(1)->setPosition(position_record_dof_1);
					mWorld->getSkeleton("cube")->getDof(2)->setPosition(position_record_dof_2);	
					cost_array[i] = mWorld->getTime() - time_start;
					//std::cin.get();
					break;
				}
			}
		}

	//	for (int i = 0; i<num_samples;i++)
	//	{
	//		std::cout<<cost_array[i]<<" "<<std::endl;
	//	}
		
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
		// guarantee the ball always has horizontal velocity
		mController->setCubeVelocity(mController->mSpeed);

		if (mController->collision_with_obstacles())
		{
			std::cout<<"collision with obstacles detected!"<<std::endl;
			mController->setCubeVelocity(0);
		}

		mController->setCubeAcceleration(MyMPC());

		SimWindow::timeStepping();
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
		std::shared_ptr<CylinderShape> cylinder = std::make_shared<CylinderShape>(
				obstacle_radius, obstacle_height);
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
		std::shared_ptr<CylinderShape> cylinder = std::make_shared<CylinderShape>(
				obstacle_radius/2.0, obstacle_height);
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
	std::shared_ptr<EllipsoidShape> ball = std::make_shared<EllipsoidShape>(
				Eigen::Vector3d(cube_length, cube_length, cube_length));
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

	glutInit(&argc, argv);
	window.initWindow(640, 480, "A toy example for Model Predictive Control");
	glutMainLoop();
}
