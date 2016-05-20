#include "MyWindow.h"

MyWindow::MyWindow(WorldPtr world)
{
	setWorld(world);
	mController = std::unique_ptr<Controller>(new Controller(mWorld->getSkeleton("mCartPole")));
}

void MyWindow::timeStepping() 
{
	//std::cout<<mWorld->getSkeleton("mCartPole")->getDof(9)->getVelocity()<<std::endl;
	SimWindow::timeStepping();
}

void MyWindow::drawSkels() {
//	glEnable(GL_LIGHTING);
//	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	SimWindow::drawSkels();
}

void MyWindow::keyboard(unsigned char _key, int _x, int _y) {
	switch(_key)
	{
		case 'a':
		mWorld->getSkeleton("mCartPole")->getDof(3)->setVelocity(1);
		break;

		case 'd':
		mWorld->getSkeleton("mCartPole")->getDof(3)->setVelocity(-1);
		break;

		case 'o':
		std::cout<<"Hold position: ";
		std::cout<<mWorld->getSkeleton("mCartPole")->getDof(0)->getPosition()<<" ";
		std::cout<<mWorld->getSkeleton("mCartPole")->getDof(1)->getPosition()<<" ";
		std::cout<<mWorld->getSkeleton("mCartPole")->getDof(2)->getPosition()<<" ";
		std::cout<<mWorld->getSkeleton("mCartPole")->getDof(3)->getPosition()<<" ";
	//	std::cout<<mWorld->getSkeleton("mCartPole")->getDof(4)->getPosition()<<" ";
	//	std::cout<<mWorld->getSkeleton("mCartPole")->getDof(5)->getPosition()<<" ";
		std::cout<<std::endl;
		break;
		
		default:
		SimWindow::keyboard(_key, _x, _y);
		break;
	}
}
