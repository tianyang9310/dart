#include "GPS.h"

namespace GPS_NSpace
{

GPS::GPS(int _numDDPIters, int _conditions):
	numDDPIters(_numDDPIters),
	conditions(_conditions),
	x0Bundle(_conditions),
	DDPBundle(_conditions)
{
	DDPIter		= 0;
}

void GPS::run()
{

	for(int i=0; i<numDDPIters; i++)
	{
		mDDP->trajopt();
		std::cout<<"########################################"<<std::endl;
		std::cout<<" Finish "<<++DDPIter<<"th DDP iteration"<<std::endl;
		std::cout<<"########################################"<<std::endl;
	}
}

}
