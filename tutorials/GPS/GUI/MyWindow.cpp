#include "MyWindow.h"

MyWindow::MyWindow(WorldPtr world, unique_ptr<GPS> _mGPS):mGPS(std::move(_mGPS))
{
	setWorld(world);
	mSnapShot = mWorld->clone();
}

void MyWindow::timeStepping() 
{
	if (mWorld->getSimFrames() == mGPS->T-1)
	{
		mGPS->run();
		setWorld(mSnapShot->clone());
		mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setPosition((mGPS->mDDP->x0)(0));
		mWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->setPosition((mGPS->mDDP->x0)(1));
		mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setVelocity((mGPS->mDDP->x0)(2));
		mWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->setVelocity((mGPS->mDDP->x0)(3));
	}
	int mSimFrameCount = mWorld->getSimFrames();
	mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setForce(mGPS->mDDP->u.col(mSimFrameCount)[0]);

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
		case 'x':
		
		break;

		default:
		SimWindow::keyboard(_key, _x, _y);
		break;
	}
}
