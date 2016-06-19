#ifndef GPS_H
#define GPS_H

#include <memory>
#include <functional>
#include "../../DDP/libDDP/DDP.h"

namespace GPS_NSpace
{

using namespace std;
using namespace DDP_NSpace;

class GPS
{
public:
	GPS(int _numDDPIters, int _conditions);
	void run();

// --------------------
	std::unique_ptr<DDP> mDDP;
	int numDDPIters;
	int DDPIter;
	int conditions;
	vector<Vector4d> x0Bundle;

};

}

#endif
