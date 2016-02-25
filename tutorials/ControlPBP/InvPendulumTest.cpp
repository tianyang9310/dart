/*

Minimal test/example of using C-PBP to balance an inverted pendulum.

*/

#include <Math.h>
#include "MathUtils.h"
#include "ControlPBP.h"
#include <iostream>
#include <fstream>
#include "Eigen/Eigen" 
#include <time.h>

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

/**
Print a line on 'not-a-numbers' to the statistics file. This information signifies the start of a new run.

@filename The filename to write to.
@startWithEmptyLine Boolean to signify if we use an empty line instead of the NaNs
*/
void printStartInfoToFile(char* filename,bool startWithEmptyLine = false){

	std::ofstream myfile;
	myfile.open (filename, std::fstream::out | std::fstream::app);
	if (!startWithEmptyLine){
		myfile << "NaN NaN";
	}
	else
	{
		myfile << "\n";
	}
	myfile << "\n";
	myfile.close();

}

/**
Print a vector the the given file. This appends a new line to the end of the file.

@filename The filename to write to.
@data A vector to append to the file as a new line.
*/
void printStatsToFile(char* filename,const Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic>& data){

	std::ofstream myfile;
	myfile.open (filename, std::fstream::out | std::fstream::app);
	if (data.cols() == 1){
		myfile << data.transpose();
	}
	else{
		myfile << data;
	}
	myfile << "\n";
	myfile.close();

}


int main(int argc, char *argv[])
{
	srand ((unsigned int)time(NULL));
	char* fileToPrintStats = "PendulumData.txt";
	char* fileForStatesAndControls = "StateAndControlData.txt";

	//global parameters
	float timeStep=1.0f/30.0f;

	//initialize the optimizer
	ControlPBP pbp;
	const int nSamples=20;	//N in the paper
	int nTimeSteps=15;		//K in the paper, resulting in a 0.5s planning horizon
	//const float PI=3.1416f;	
	const int nStateDimensions=2;
	const int nControlDimensions=1;
	float minControl=-1000;	//lower sampling bound
	float maxControl=1000;		//upper sampling bound
	float controlMean=0;	//we're using torque as the control, makes sense to have zero mean
	//Square root of the diagonal elements of C_u in the paper, i.e., stdev of a Gaussian prior for control.
	//Note that the optimizer interface does not have the C_u as a parameter, and instead uses meand and stdev arrays as parameters. 
	//The 3D character tests compute the C_u on the Unity side to reduce the number of effective parameters, and then compute the arrays based on it as described to correspond to the products \sigma_0 C_u etc.
	float C=10;	
	float controlStd=1.0f*C;	//sqrt(\sigma_{0}^2 C_u) of the paper (we're not explicitly specifying C_u as u is a scalar here). In effect, a "tolerance" for torque minimization in this test
	float controlDiffStd=1.0f*C;	//sqrt(\sigma_{1}^2 C_u) in the paper. In effect, a "tolerance" for angular jerk minimization in this test
	float controlDiffDiffStd=100.0f*C; //sqrt(\sigma_{2}^2 C_u) in the paper. A large value to have no effect in this test.
	float stateStd[2]={0.1f*PI,PI};	//square roots of the diagonal elements of Q in the paper
	//float* stateStd = NULL;
	float mutationScale=0.25f;		//\sigma_m in the paper
	pbp.init(nSamples,nTimeSteps,nStateDimensions,nControlDimensions,&minControl,&maxControl,&controlMean,&controlStd,&controlDiffStd,&controlDiffDiffStd,mutationScale,stateStd);

	//set further params: portion of "no prior" samples, resampling threshold, whether to use the backwards smoothing pass, and the regularization of the smoothing pass
	pbp.setParams(0.1f,0.5f,true,0.001f);  

	//allocate simulation states
	float state[nSamples][nStateDimensions];
	float nextState[nSamples][nStateDimensions];
	float masterState[2]={PI,0};

	printStartInfoToFile(fileToPrintStats);

	//run the algorithm for 90 steps (3 seconds)
	for (int n=0; n<90; n++)
	{
		//signal the start of new C-PBP iteration
		pbp.startIteration(true,masterState);
		//init all random walker states to the master state
		for (int i=0; i<nSamples; i++)
		{
			memcpy(&state[i],masterState,sizeof(float)*nStateDimensions);
		}

		//simulate forward 
		for (int k=0; k<nTimeSteps; k++)
		{
			//signal the start of a planning step
			pbp.startPlanningStep(k);
			//NOTE: for multithreaded operation, one would typically run each iteration of the following loop in a separate thread. 
			//The getControl(), getPreviousSampleIdx(), and updateResults() methods of the optimizer are thread-safe. Regarding the physics simulation,
			//one would typically have a separate instance of the whole physics world for each thread, with full physics state stored/loaded similar to the state and nextState arrays here.
			for (int i=0; i<nSamples; i++)
			{
				//get control from C-PBP
				float control;
				pbp.getControl(i,&control);

				//get the mapping from this to previous state (affected by resampling operations)
				int previousStateIdx=pbp.getPreviousSampleIdx(i);

				//simulate to get next state.
				simulatePendulum(state[previousStateIdx],control,nextState[i],timeStep);

				//evaluate state cost
				float cost=squared(nextState[i][ANGLE]*10.0f) + squared(nextState[i][AVEL]);

				//store the state and cost to C-PBP. Note that in general, the stored state does not need to contain full simulation state as in this simple case.
				//instead, one may use arbitrary state features
				pbp.updateResults(i,&control,nextState[i],cost);
			}
			//update all states, will be used at the next step
			memcpy(state,nextState,sizeof(state));

			//signal the end of the planning step. this normalizes the state costs etc. for the next step
			pbp.endPlanningStep(k);

		}
		//signal the end of an iteration. this also executes the backwards smoothing pass
		pbp.endIteration();

//		Eigen::MatrixXf tmpSC = pbp.stateAndControlToMatrix();
//		printStatsToFile(fileForStatesAndControls,tmpSC);

		//deploy the best control found
		float control;
		pbp.getBestControl(0,&control);

		simulatePendulum(masterState,control,masterState,timeStep);


		//print output, both angle and avel should converge to 0, with some sampling noise remaining.
		printf("Pendulum angle %1.3f, avel %1.3f\n",masterState[ANGLE],masterState[AVEL]);
		Eigen::VectorXf stateTmp = Eigen::VectorXf::Zero(2);
		stateTmp(0) = masterState[ANGLE];
		stateTmp(1) = masterState[AVEL];
		printStatsToFile(fileToPrintStats,stateTmp);
	} 
	return 0;
}
