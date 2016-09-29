#ifndef MYDANTZIGLCPSOLVER
#define MYDANTZIGLCPSOLVER

#include <cstddef>
#include <iomanip>
#include <iostream>

#include "dart/constraint/LCPSolver.h"
#include "dart/constraint/DantzigLCPSolver.h"
#include "dart/constraint/ConstraintBase.h"
#include "dart/constraint/ConstrainedGroup.h"

#include "dart/config.h"
#include "dart/common/Console.h"
#include "dart/lcpsolver/Lemke.h"
#include "dart/lcpsolver/lcp.h"

using namespace dart::constraint;

class MyDantzigLCPSolver : public dart::constraint::DantzigLCPSolver
{
public:
    MyDantzigLCPSolver(double _timestep);
    virtual ~MyDantzigLCPSolver(){};

    void solve(ConstrainedGroup* _group) override;
protected:
    void print(size_t _n, double* _A, double* _x, double* _lo, double* _hi, double* _b, double* w, int* _findex);
    

};


#endif // MYDANTZIGLCPSOLVER
