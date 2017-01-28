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
  addExtForce();
    
    dart::gui::SimWindow::timeStepping();
    if (mWorld->getSimFrames() == 2500) {
      keyboard(' ',0,0);
      std::cout << "mbox pos " << mWorld->getSkeleton("mBox")->getPositions().transpose() << std::endl; 
    }
}

void MyWindow::addExtForce()
{
    // mWorld->getSkeleton("mBox")->getBodyNode("BodyNode_1")->addExtTorque(Eigen::Vector3d::Random()*100);
    Eigen::Vector3d extForce;
    extForce.setZero();
    extForce(0) = 1.2e1; //*std::cos(45.0/180.0*DART_PI);
    // extForce(2) = 1.2e1*std::sin(45.0/180.0*DART_PI);
    mWorld->getSkeleton("mBox")->getBodyNode("BodyNode_1")->addExtForce(extForce);
}

void MyWindow::keyboard(unsigned char _key, int _x, int _y)
{
    switch (_key)
    {
        case 'a':
            addExtForce();
            break;
        default:
            dart::gui::SimWindow::keyboard(_key,_x,_y);
    }
    
}

void MyWindow::drawSkels()
{
    // Make sure lighting is turned on and that polygons get filled in
    glEnable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    
    dart::gui::SimWindow::drawSkels();
}
