#ifndef MYDANTZIGLCPSOLVER
#define MYDANTZIGLCPSOLVER

#include <cstddef>
#include <iomanip>
#include <iostream>

#include "dart/dart.h"

// #include "dart/constraint/LCPSolver.h"
// #include "dart/constraint/DantzigLCPSolver.h"
// #include "dart/constraint/ConstraintBase.h"
// #include "dart/constraint/ConstrainedGroup.h"
//
// #include "dart/config.h"
// #include "dart/common/Console.h"
#include "dart/lcpsolver/Lemke.h"
// #include "dart/lcpsolver/lcp.h"

#ifndef DART_CONTACT_CONSTRAINT_EPSILON
#define DART_CONTACT_CONSTRAINT_EPSILON  1e-6
#endif

#ifndef DART_EPSILON
#define DART_EPSILON (1.0E-6)
#endif
using namespace dart::constraint;

class MyDantzigLCPSolver : public DantzigLCPSolver
{
public:
    MyDantzigLCPSolver(double _timestep,int _totalDOF);
    virtual ~MyDantzigLCPSolver(){};
    
    void solve(ConstrainedGroup* _group) override;
protected:
    void print(size_t _n, double* _A, double* _x, double* _lo, double* _hi, double* _b, double* w, int* _findex);
    
    /// Return true if the matrix is symmetric
    bool isSymmetric(size_t _n, double* _A);
    
    /// Return true if the diagonla block of matrix is symmetric
    bool isSymmetric(size_t _n, double* _A, size_t _begin, size_t _end);

    Eigen::MatrixXd getTangentBasisMatrix(const Eigen::Vector3d& _n);

    int totalDOF;

    int numBasis;
};


#endif // MYDANTZIGLCPSOLVER
