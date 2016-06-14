#include <memory>
#include "libGPS/GPS.h"

using namespace std;
using namespace GPS_NSpace;

int main()
{
	unique_ptr<GPS> mGPS = unique_ptr<GPS>(new GPS());

//	debugging agent class
//  (dynamic_pointer_cast<DCPagent>(mGPS->mAgent))->sample();
//	mGPS->mAgent->get_samples();
//	mGPS->mAlgorithm->iteration();

	return 0;
}
