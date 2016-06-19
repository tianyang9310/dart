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
	GPS();
	void run();

// --------------------
	std::unique_ptr<DDP> mDDP;
	int numDDPIters;
	int DDPIter;

};

}

#endif
