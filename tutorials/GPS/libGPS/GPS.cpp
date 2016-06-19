#include "GPS.h"

namespace GPS_NSpace
{

GPS::GPS(int _numDDPIters, int _conditions):
	numDDPIters(_numDDPIters),
	conditions(_conditions),
	x0Bundle(_conditions)
{
	DDPIter		= 0;
}

void GPS::run()
{
	cout<<x0Bundle[0]<<endl;
	cout<<x0Bundle[1]<<endl;
	cout<<x0Bundle[2]<<endl;
	cout<<x0Bundle[3]<<endl;
	cout<<x0Bundle[4]<<endl;
	cin.get();
	for(int i=0; i<numDDPIters; i++)
	{
		mDDP->trajopt();
		std::cout<<"########################################"<<std::endl;
		std::cout<<" Finish "<<++DDPIter<<"th DDP iteration"<<std::endl;
		std::cout<<"########################################"<<std::endl;
	}
}

}
