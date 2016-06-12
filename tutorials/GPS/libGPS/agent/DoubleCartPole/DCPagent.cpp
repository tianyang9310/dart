#include "DCPagent.h"

namespace GPS_NSpace
{

DCPagent::DCPagent(hyperparameters& _mHyperparameters):agent(_mHyperparameters)
{
	rendering = true;
	mHyperparameters.tupleBool.insert(pair<string,bool>("rendering",rendering));

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
