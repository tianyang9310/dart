#ifndef MYWORLD
#define MYWORLD

#include "MyConstraintSolver.h"
#include "dart/simulation/World.h"

namespace CntctLrnin {

using namespace dart::simulation;

class MyWorld : public World {
  public:
  MyWorld(const std::string& _name = "world");
  MyWorld(const WorldPtr baseWorldPtr);
  virtual ~MyWorld();
};
}

#endif  // MYWORLD
