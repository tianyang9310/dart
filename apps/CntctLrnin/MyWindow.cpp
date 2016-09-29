#include "MyWindow.h"

using namespace std;

MyWindow::MyWindow(dart::simulation::WorldPtr world)
{
    setWorld(world);
    mCollisionDetector = std::unique_ptr<dart::collision::CollisionDetector>(mWorld->getConstraintSolver()->getCollisionDetector());
}

MyWindow::~MyWindow() 
{
    
}

void MyWindow::timeStepping()
{
    int numCntct = mCollisionDetector->getNumContacts();
    if (numCntct>0)
    {
        cout<<"There are "<<numCntct<<" contact points"<<endl;
        // retrieve A and p in LCP (for only normal part, assuming frictionless)
        
    }
    
    dart::gui::SimWindow::timeStepping();
}

void MyWindow::keyboard(unsigned char _key, int _x, int _y)
{
    
    dart::gui::SimWindow::keyboard(_key,_x,_y);
}

void MyWindow::drawSkels()
{
    // Make sure lighting is turned on and that polygons get filled in
    glEnable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    
    dart::gui::SimWindow::drawSkels();
}
