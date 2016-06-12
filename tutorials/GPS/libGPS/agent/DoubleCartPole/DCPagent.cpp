#include "DCPagent.h"

namespace GPS_NSpace
{

DCPagent::DCPagent()
{
	rendering = true;
	// update config
	// agent() // no arugment base construtor will be called before derived class constructor, thus there is no need to called agent() again, except agent accept other arguements
}

void DCPagent::sample()
{
	cout<<"DCP_agent sample"<<endl;
}

void DCPagent::get_samples()
{
	cout<<"DCP_agent get samples"<<endl;
}


}
