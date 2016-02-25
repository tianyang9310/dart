/*

Minimal test/example of using the kD-tree Sequential Monte Carlo (Hämäläinen et al. 2014 SIGGRAPH paper) to balance an inverted pendulum. In contrast to the paper,
we have here added a prior for the controls - before drawing a sample from the proposal Gaussian provided by the sampler, we multiply the proposal with the prior.

*/

#include <Math.h>
#include "MathUtils.h"
#include "RandomForestSampler.h"
#include "ClippedGaussianSampling.h"
#include <stdio.h>

using namespace AaltoGames;

static const int ANGLE=0;  //state variable index
static const int AVEL=1;	//state variable index

void simulatePendulum(const float *state, float controlTorque, float *nextState, float timeStep)
{
	//Compute new angle and angular velocity using basic Euler integration
	const float length=1.0f;
	float gravityTorque=fabs(sinf(state[ANGLE])*length)*9.81f;
	nextState[AVEL]=state[AVEL]+(gravityTorque+controlTorque)*timeStep;  //assuming moment of inertia 1
	nextState[ANGLE]=state[ANGLE]+nextState[AVEL]*timeStep;
}


void main(int argc, char *argv)
{
	//global parameters
	float timeStep=1.0f/30.0f;

	//initialize the optimizer
	RandomForestSampler sampler;
	const int nSamples=100;	
	const int nForwardSimulationSteps=15;	
	const int nStateDimensions=2;
	const int nControlDimensions=1;
	const int nSamplingDimensions=nControlDimensions*nForwardSimulationSteps;

	//init sampling bounds
	float minControl[nSamplingDimensions],maxControl[nSamplingDimensions];
	for (int i=0; i<nSamplingDimensions; i++)
	{
		minControl[i]=-100;
		maxControl[i]=100;
	}
	sampler.setSize(10,nSamples*2);
	sampler.init(nSamplingDimensions,0,minControl,maxControl);

	//set parameters for basic importance sampling, no greediness or local refinement
	sampler.setParams(0.5f,1.0f,0,nSamples/4,0,0);

	//allocate simulation states
	float state[nStateDimensions];
	float masterState[nStateDimensions]={PI,0};

	//run the algorithm for 200 steps
	for (int n=0; n<200; n++)
	{
		//signal that old fitness values are invalid
		sampler.landscapeUpdated();
		//prevent the sample trees from growing infinitely
		sampler.prune(nSamples/4);	

		//loop over all samples
		for (int i=0; i<nSamples; i++)
		{
			//initialize sample state to the master state
			memcpy(state,masterState,sizeof(state));
			//get the proposal Gaussian for this sample
			float proposalMean[nSamplingDimensions],proposalSd[nSamplingDimensions];
			sampler.getProposalGaussian(proposalMean,proposalSd);

			//loop over forward simulation steps, sampling control values and accumulating the total cost over the trajectory
			float totalCost=0;
			float sample[nSamplingDimensions];
			for (int k=0; k<nForwardSimulationSteps; k++)
			{
				//combine prior with the proposal. note: prior can be arbitrary and change for each sample, e.g., based on state. 
				//here we simply use a constant prior Gaussian that favors small torques.
				float priorMean=0,priorSd=10; 
				float controlMean,controlSd;
				productNormalDist(priorMean,priorSd,proposalMean[k],proposalSd[k],controlMean,controlSd);

				//draw the control (here: pendulum torque)
				float control=randGaussianClipped(controlMean,controlSd,minControl[k],maxControl[k]);

				//store the control as part of the generated sample vector
				sample[k]=control;

				//simulate to get next state.
				simulatePendulum(state,control,state,timeStep);

				//evaluate costs
				float stateCost=squared(state[ANGLE]) + squared(state[AVEL]/10.0f);

				//accumulate total cost
				totalCost+=stateCost;
			}
			//convert cost to fitness (sample weight)
			double fitness=exp(-0.5f*totalCost);

			//store evaluated sample
			sampler.putSample(sample,NULL,fitness);
		} //for each sample
		//deploy the first control of the best sampled trajectory
		const float *bestSample=sampler.getBestSample();
		simulatePendulum(masterState,bestSample[0],masterState,timeStep);

		//shift the control vectors so that we get a prior that works at the next time step
		SampleArray *samples=sampler.getAllSamples();
		for (int i=0; i<samples->getCount(); i++)
		{
			OptimizerSample *sample=samples->getSample(i);
			float *values=sample->getInputs();
			for (int k=0; k<nForwardSimulationSteps-1; k++)
			{
				values[k]=values[k+1];
			}
			values[nForwardSimulationSteps-1]=0; //init the last control value to prior mean (no torque)
		}

		//print output, both angle and avel should converge to 0, with some sampling noise remaining.
		printf("Pendulum angle %1.3f, avel %1.3f\n",masterState[ANGLE],masterState[AVEL]);
	} 
	getchar();
}
