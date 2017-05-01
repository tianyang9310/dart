#ifndef CNTCTLRNIN_PARAM_H
#define CNTCTLRNIN_PARAM_H

#include "addSkeles.h"

/// Which solver to use default is ODE
#define LEMKE_SOLVER

//-----------------------------------------------------------------------------
/// unit tests
// #define UNIT_TEST
#ifdef UNIT_TEST

#define STRAIGHT_PUSH

// #define STATIC_SLOPE

#define NUMBODYNODES 1

#else

#define NUMBODYNODES 5

#endif

//-----------------------------------------------------------------------------

/// Use NN model to predict
// #define CAFFE_LCP

/// Number of basis
#define NUMBASIS 4

/// Print precision
#define PRECISION 20

/// Shape of body bode
#define SHAPE mShapeType::cube

#define RANDOM_DURATION 2
#define PERIOD 10

//-----------------------------------------------------------------------------

/// Solver Config of PGS
#define PGS_MAX_ITER 1e5
#define PGS_ZERO 1e-12

#endif
