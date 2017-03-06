#ifndef CNTCTLRNIN_PARAM_H
#define CNTCTLRNIN_PARAM_H

#include "addSkeles.h"

/// Which solver to use default is ODE
#define LEMKE_SOLVER

/// unit tests
#define UNIT_TEST
#ifdef UNIT_TEST  

#define STRAIGHT_PUSH

// #define STATIC_SLOPE

// #define DYNAMIC_SLOPE

#define NUMBODYNODES 1

#else 

#define NUMBODYNODES 6

#endif

/// Number of basis
#define NUMBASIS 8

/// Print precision
#define PRECISION 20



/// Shape of body bode
#define SHAPE mShapeType::cube

#endif
