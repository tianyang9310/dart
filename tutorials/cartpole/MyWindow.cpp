#include "MyWindow.h"
//#define mPOINTER_DEBUG
//#define mDOFSTAT

MyWindow::MyWindow(WorldPtr world)
{
	setWorld(world);
	mSnapshot   = mWorld->clone();
	mController = std::unique_ptr<Controller>(new Controller(mWorld->getSkeleton("mCartPole")));
	mDDP_iter   = 0;
}

inline void MyWindow::mPointer_Debug()
{
#ifdef mPOINTER_DEBUG
	std::cout<<&mWorld<<std::endl;
	std::cout<<mWorld<<std::endl;
	std::cout<<mSnapshot<<std::endl;
	std::cout<<"mWorld pointer reference count is "<<mWorld.use_count()<<std::endl;
#endif
}

inline void MyWindow::mDofStat()
{
#ifdef mDOFSTAT
	for (size_t i = 0; i<mWorld->getSkeleton("mCartPole")->getNumDofs();i++)
	{
		std::cout<<mWorld->getSkeleton("mCartPole")->getDof(i)->getName()<<"	position: "<<mWorld->getSkeleton("mCartPole")->getDof(i)->getPosition()<<"		velocity: "<<mWorld->getSkeleton("mCartPole")->getDof(i)->getVelocity()<<"		acceleration: "<<mWorld->getSkeleton("mCartPole")->getDof(i)->getAcceleration()<<std::endl;
	}
#endif
}

void MyWindow::timeStepping() 
{
	std::cout<<mWorld->getTime()<<std::endl;
	if (mWorld->getSimFrames()==mController->mDDP->T-1)
	{
		std::cout<<"########################################"<<std::endl;
		std::cout<<"       Finish "<<++mDDP_iter<<"th DDP iteration        "<<std::endl;
		std::cout<<"########################################"<<std::endl;
		mPointer_Debug();
		mDofStat();
		setWorld(mSnapshot->clone());
		mDofStat();
		mPointer_Debug();
		std::cin.get();
	}
	// apply external force via u[i]
	// std::cout<<mController->mDDP->u.col(mWorld->getSimFrames())<<std::endl;
	mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setForce(mController->mDDP->u.col(mWorld->getSimFrames())[0]); 
	SimWindow::timeStepping();
}

void MyWindow::drawSkels() 
{
//	glEnable(GL_LIGHTING);
//	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	SimWindow::drawSkels();
}

void MyWindow::keyboard(unsigned char _key, int _x, int _y) {
	switch(_key)
	{
		case 'a':
		mWorld->getSkeleton("mCartPole")->getDof(0)->setForce(100);
		break;

		case 'd':
		mWorld->getSkeleton("mCartPole")->getDof(0)->setForce(-100);
		break;

		case 'r':
		mWorld->getSkeleton("mCartPole")->getDof(0)->setPosition(0);
		break;

	//	case 'o':
	//	std::cout<<mWorld->getSkeleton("mCartPole")->getBodyNode(0)->getTransform().matrix()<<std::endl;
	//	break;

		default:
		SimWindow::keyboard(_key, _x, _y);
		break;
	}
}
