#include "DCPagent.h"
#include "../../utility/utility.h"

namespace GPS_NSpace
{

extern hyperparameters mHyperparameters;

DCPagent::DCPagent()
{
	// placeholders
}
DCPagent::DCPagent(bool placeholders):agent(true)
{
	// update mHyperparameters
	rendering = true;
	mHyperparameters.tupleBool.insert(pair<string,bool>("rendering",rendering));

	// setup_conditions, augment some variable to conditions, this method is not necessary since we can setup it in hyperparamters
	// ('x0', 'x0var', 'pos_body_idx', 'pos_body_offset', 'noisy_body_idx', 'noisy_body_var') populate to vector as vector
	
	//_setup_world();
}

void DCPagent::sample()
{
	cout<<"DCP_agent sample"<<endl;
}

void DCPagent::get_samples()
{
	cout<<"DCP_agent get samples"<<endl;
}

void DCPagent::_setup_world()
{
	WorldPtr mWorld = std::make_shared<World>();
	WorldSetup(mWorld);

	window = MyWindow(mWorld);

	int argc = 0;
	char* argv[] = {""};
	glutInit(&argc, argv);
	window.initWindow(1024, 768, "Vehicle");
	glutMainLoop();
}


}
