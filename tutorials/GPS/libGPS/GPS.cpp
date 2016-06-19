#include "GPS.h"

namespace GPS_NSpace
{

GPS::GPS()
{
	numDDPIters = 20;
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
