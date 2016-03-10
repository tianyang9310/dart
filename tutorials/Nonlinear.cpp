/*
 * Nonlinear Project
 * Yang Tian
 * Email: tianyang9310@gmail.com
 * Date: Mar. 8th 2016
*/


#include "dart/dart.h"
const double default_speed_increment = 0.5;
const int default_ik_iterations = 4500;
const double default_force =  50.0; // N
const int default_countdown = 100;  // Number of timesteps for applying force

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;
using namespace dart::utils;
using namespace dart::math;

class Controller
{
public:
  /// Constructor
  Controller(const SkeletonPtr& biped)
    : mBiped(biped),
      mPreOffset(0.0),
      mSpeed(0.0)
  {
    int nDofs = mBiped->getNumDofs();
    
    mForces = Eigen::VectorXd::Zero(nDofs);
    
    mKp = Eigen::MatrixXd::Identity(nDofs, nDofs);
    mKd = Eigen::MatrixXd::Identity(nDofs, nDofs);
  
    for(size_t i = 0; i < 6; ++i)
    {
      mKp(i, i) = 0.0;
      mKd(i, i) = 0.0;
    }

    for(size_t i = 6; i < biped->getNumDofs(); ++i)
    {
      mKp(i, i) = 1000;
      mKd(i, i) = 50;
    }

    setTargetPositions(mBiped->getPositions());
  }
  
  /// Reset the desired dof position to the current position
  void setTargetPositions(const Eigen::VectorXd& pose)
  {
    mTargetPositions = pose;
  }

  /// Clear commanding forces
  void clearForces()
  {
    mForces.setZero();
  }
  
  /// Add commanding forces from PD controllers
  void addPDForces()
  {
    // Lesson 2
	  Eigen::VectorXd q = mBiped->getPositions();
	  Eigen::VectorXd dq = mBiped->getVelocities();

	  Eigen::VectorXd p = -mKp * (q- mTargetPositions);
	  Eigen::VectorXd d = -mKd * dq;

	  mForces += p + d;
	  mBiped->setForces(mForces);
  }

  /// Add commanind forces from Stable-PD controllers
  void addSPDForces()
  {
    // Lesson 3
	Eigen::VectorXd q = mBiped->getPositions();
    Eigen::VectorXd dq = mBiped->getVelocities();

    Eigen::MatrixXd invM = (mBiped->getMassMatrix() + mKd * mBiped->getTimeStep()).inverse();
    Eigen::VectorXd p = -mKp * (q + dq * mBiped->getTimeStep() - mTargetPositions);
    Eigen::VectorXd d = -mKd * dq;
    Eigen::VectorXd qddot = invM * (-mBiped->getCoriolisAndGravityForces() + p + d + mBiped->getConstraintForces());

    mForces += p + d - mKd * qddot * mBiped->getTimeStep();
    mBiped->setForces(mForces);
  }
  
  /// add commanding forces from ankle strategy
  void addAnkleStrategyForces()
  {
    // Lesson 4
	// to prove that we do have the ankle stratgy 
	  std::cout<<"Current angle between foot and shin"<<std::endl;
	  std::cout<<mBiped->getDof("j_heel_left_1")->getPosition()<<std::endl;
	  std::cout<<mBiped->getDof("j_heel_right_1")->getPosition()<<std::endl;

	Eigen::Vector3d COM = mBiped->getCOM();
	Eigen::Vector3d offset(0.03, 0, 0);
	Eigen::Vector3d COP = mBiped->getBodyNode("h_heel_left")->getTransform() * offset;
	double diff = COM[0] - COP[0] ;

	Eigen::Vector3d dCOM = mBiped->getCOMLinearVelocity();
	Eigen::Vector3d dCOP = mBiped->getBodyNode("h_heel_left")->getLinearVelocity(offset);
    double dDiff = dCOM[0] - dCOP[0];

	// both left and right heel use the same deviation computed above because now we assume 
	// the robots' two legs work in an identical fashion.
	int leftHeelIndex = mBiped->getDof("j_heel_left_1")->getIndexInSkeleton();
	int rightHeelIndex = mBiped->getDof("j_heel_right_1")->getIndexInSkeleton();
	if (diff < 10.0 && diff > 0.0)
	{
		double k1 = 10.0;
		double kd = 0.50;
		mForces[leftHeelIndex] += -k1 * diff -kd *dDiff;
		mForces[rightHeelIndex] += -k1 * diff -kd *dDiff;
	}
	else if (diff > -20.0 && diff < 0.0 )
	{
		double k1 = 100.0;
		double kd = 5.0;
		mForces[leftHeelIndex] += -k1 * diff -kd *dDiff;
		mForces[rightHeelIndex] += -k1 * diff -kd *dDiff;
	}
	mBiped->setForces(mForces);
  }
  
  // Send velocity commands on wheel actuators
  void setWheelCommands()
  {
    // Lesson 6
  }
  
  void changeWheelSpeed(double increment)
  {
    mSpeed += increment;
    std::cout << "wheel speed = " << mSpeed << std::endl;
  }
  
protected:
  /// The biped Skeleton that we will be controlling
  SkeletonPtr mBiped;
  
  /// Joint forces for the biped (output of the Controller)
  Eigen::VectorXd mForces;
  
  /// Control gains for the proportional error terms in the PD controller
  Eigen::MatrixXd mKp;

  /// Control gains for the derivative error terms in the PD controller
  Eigen::MatrixXd mKd;

  /// Target positions for the PD controllers
  Eigen::VectorXd mTargetPositions;
  
  /// For ankle strategy: Error in the previous timestep
  double mPreOffset;
  
  /// For velocity actuator: Current speed of the skateboard
  double mSpeed;
};

class MyWindow : public SimWindow
{
public:
  /// Constructor
  MyWindow(const WorldPtr& world)
  : mForceCountDown(0),
    mPositiveSign(true)
  {
    setWorld(world);
    
    mController = std::unique_ptr<Controller>
        (new Controller(mWorld->getSkeleton("biped")));
  }
  
  /// Handle keyboard input
  void keyboard(unsigned char key, int x, int y) override
  {
    switch(key)
    {
      case ',':
        mForceCountDown = default_countdown;
        mPositiveSign = false;
        break;
      case '.':
        mForceCountDown = default_countdown;
        mPositiveSign = true;
        break;
      case 'a':
        mController->changeWheelSpeed(default_speed_increment);
        break;
      case 's':
        mController->changeWheelSpeed(-default_speed_increment);
        break;
      default:
        SimWindow::keyboard(key, x, y);
    }
  }

  void timeStepping() override
  {
    mController->clearForces();
    
	// Lesson 2
	// mController->addPDForces();

    // Lesson 3
    mController->addSPDForces();

    // Lesson 4
    mController->addAnkleStrategyForces();
    
    // Lesson 6
    mController->setWheelCommands();
    
    // Apply body forces based on user input, and color the body shape red
    if(mForceCountDown > 0)
    {
      BodyNode* bn = mWorld->getSkeleton("biped")->getBodyNode("h_pelvis");
      const ShapePtr& shape = bn->getVisualizationShape(0);
      shape->setColor(dart::Color::Red());
      
      if(mPositiveSign)
        bn->addExtForce(default_force * Eigen::Vector3d::UnitX(),
                        bn->getCOM(), false, false);
      else
        bn->addExtForce(-default_force*Eigen::Vector3d::UnitX(),
                        bn->getCOM(), false, false);
      
      --mForceCountDown;
    }
	std::cout<<"The COM is "<<std::endl<<mWorld->getSkeleton("biped")->getCOM()<<std::endl;
    
    // Step the simulation forward
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
  
  /// Number of iterations before clearing a force entry
  int mForceCountDown;
  
  /// Whether a force should be applied in the positive or negative direction
  bool mPositiveSign;
  
};

// Load a biped model and enable joint limits and self-collision
SkeletonPtr loadBiped()
{
  // Lesson 1
  
  // Create the world with a skeleton
  WorldPtr world = SkelParser::readWorld(DART_DATA_PATH"skel/Nonlinear_Biped.skel");
  assert(world != nullptr);

  SkeletonPtr biped = world->getSkeleton("biped");

  // To make sure the bipedal robots act like human, 
  // 1. enforce joint limit
  // 2. enable self-collision
  for (size_t i=0; i<biped->getNumJoints(); ++i)
  {
	  biped->getJoint(i)->setPositionLimited(true);
  }
  
  // Enable self-collision detection in DART
  // By default DART doesn't check the self collision
  biped->enableSelfCollision();

  // set initial position of the robot
  biped->setPosition(biped->getDof("j_thigh_left_z")->getIndexInSkeleton(), 0.15);
  biped->setPosition(biped->getDof("j_thigh_right_z")->getIndexInSkeleton(), 0.15);
  biped->setPosition(biped->getDof("j_shin_left")->getIndexInSkeleton(), -0.4);
  biped->setPosition(biped->getDof("j_shin_right")->getIndexInSkeleton(), -0.4);
  biped->setPosition(biped->getDof("j_heel_left_1")->getIndexInSkeleton(), 0.25);
  biped->setPosition(biped->getDof("j_heel_right_1")->getIndexInSkeleton(), 0.25); 

  return biped;
}

// Load a skateboard model and connect it to the biped model via an Euler joint
void modifyBipedWithSkateboard(SkeletonPtr /*biped*/)
{
  // Lesson 5
}

// Set the actuator type for four wheel joints to "VELOCITY"
void setVelocityAccuators(SkeletonPtr /*biped*/)
{
  // Lesson 6
}

// Solve for a balanced pose using IK
Eigen::VectorXd solveIK(SkeletonPtr biped)
{
  // Lesson 7
  Eigen::VectorXd newPose = biped->getPositions();
  return newPose;
}

SkeletonPtr createFloor()
{
  SkeletonPtr floor = Skeleton::create("floor");
  
  // Give the floor a body
  BodyNodePtr body =
      floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;
  
  // Give the body a shape
  double floor_width = 10.0;
  double floor_height = 0.01;
  std::shared_ptr<BoxShape> box(
      new BoxShape(Eigen::Vector3d(floor_width, floor_height, floor_width)));
  box->setColor(dart::Color::Gray(0.2));
  
  body->addVisualizationShape(box);
  body->addCollisionShape(box);
  
  // Put the body into position
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.0, -1.0, 0.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);
  
  return floor;
}

int main(int argc, char* argv[])
{
  SkeletonPtr floor = createFloor();

  // Lesson 1
  SkeletonPtr biped = loadBiped();
  
  // Lesson 5
  modifyBipedWithSkateboard(biped);

  // Lesson 6
  setVelocityAccuators(biped);

  // Lesson 7
  Eigen::VectorXd balancedPose = solveIK(biped);
  biped->setPositions(balancedPose);
  
  WorldPtr world = std::make_shared<World>();
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

#ifdef HAVE_BULLET_COLLISION
  world->getConstraintSolver()->setCollisionDetector(
          new dart::collision::BulletCollisionDetector());
#endif
  
  world->addSkeleton(floor);
  world->addSkeleton(biped);
  
  // Create a window for rendering the world and handling user input
  MyWindow window(world);

  // Print instructions
  std::cout << "'.': forward push" << std::endl;
  std::cout << "',': backward push" << std::endl;
  std::cout << "'s': increase skateboard forward speed" << std::endl;
  std::cout << "'a': increase skateboard backward speed" << std::endl;
  std::cout << "space bar: simulation on/off" << std::endl;
  std::cout << "'p': replay simulation" << std::endl;
  std::cout << "'v': Turn contact force visualization on/off" << std::endl;
  std::cout << "'[' and ']': replay one frame backward and forward" << std::endl;
 
  // Initialize glut, initialize the window, and begin the glut event loop
  glutInit(&argc, argv);
  window.initWindow(1068, 768, "Nonlinear Project");
  glutMainLoop();
}
