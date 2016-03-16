/*************************************************************************
    > File Name: MyWindow.cpp
    > Author: Yang Tian
    > Mail: tianyang9310@gmail.com 
    > Created Time: Wed 16 Mar 2016 11:53:26 AM EDT
 ************************************************************************/

#include "MyWindow.h"
namespace toyexample{

MyWindow::MyWindow(WorldPtr world)
{
	setWorld(world);	
	mController =  std::unique_ptr<Controller>(new Controller(mWorld->getSkeleton("cube")));
}

void MyWindow::timeStepping() 
{
	SimWindow::timeStepping();
}




} // namespace toyexample
