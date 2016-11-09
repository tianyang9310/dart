#include "MyWorld.h"

namespace dart{
namespace simulation{

MyWorld::MyWorld(const std::string& _name):World(_name)
{
    mConstraintSolver = new dart::constraint::MyConstraintSolver(mTimeStep);
}

MyWorld::~MyWorld()
{

}

}
}
