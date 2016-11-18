#ifndef MYWORLD
#define MYWORLD

#include "MyConstraintSolver.h"
#include "dart/simulation/World.h"

namespace dart {
namespace simulation {

class MyWorld : public World {
  public:
  MyWorld(const std::string& _name = "world");
  virtual ~MyWorld();
};
}
}

#endif  // MYWORLD
