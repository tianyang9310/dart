#include "MyWorld.h"

namespace CntctLrnin {

MyWorld::MyWorld(const std::string& _name) : World(_name) {
  mConstraintSolver = new MyConstraintSolver(mTimeStep);
}

MyWorld::~MyWorld() {}
}
