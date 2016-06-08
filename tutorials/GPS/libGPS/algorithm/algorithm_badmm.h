#ifndef ALGORITHM_BADMM_H
#define ALGORITHM_BADMM_H

#include <iostream>
#include "algorithm.h"

namespace GPS_NSpace
{
using namespace std;

class algorithm_badmm : public algorithm
{
public:
	algorithm_badmm();
	void iteration();

};

}

#endif
