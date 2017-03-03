#include "MyWorld.h"

namespace CntctLrnin {

MyWorld::MyWorld(const std::string& _name) : World(_name) {
#ifdef LEMKE_SOLVER
  mConstraintSolver = new MyConstraintSolver(mTimeStep);
#endif
}

MyWorld::MyWorld(const WorldPtr baseWorld) : World(*baseWorld) {
#ifdef LEMKE_SOLVER
  mConstraintSolver = new MyConstraintSolver(mTimeStep);
#endif
};

MyWorld::~MyWorld() {}
}
