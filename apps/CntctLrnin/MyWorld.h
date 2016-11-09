#ifndef MYWORLD
#define MYWORLD

#include "dart/simulation/World.h"
#include "MyConstraintSolver.h"

namespace dart{
namespace simulation{

class MyWorld : public World
{
public:
    MyWorld(const std::string& _name = "world");
    virtual ~MyWorld();

};

}
}

#endif // MYWORLD
