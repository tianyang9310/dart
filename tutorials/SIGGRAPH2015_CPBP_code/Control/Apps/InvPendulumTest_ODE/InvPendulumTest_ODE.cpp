/*

Minimal test/example of using C-PBP and ODE to balance an inverted pendulum. The difference to InvPendulumTest.cpp is that here
we are using a black box physics engine for the simulation, and not passing the full engine state to the optimizer. Instead, only a 
vector of relevant state is passed.

Note that this example uses our UnityOde wrapper, but you might want to implement your state saving/loading and handling of multiple world copies yourself.

*/

#include <Math.h>
#include "MathUtils.h"
#include "ControlPBP.h"
#include "UnityOde.h"
#include <ode/ode.h>
#include <stdio.h>

using namespace AaltoGames;

static const int ANGLE=0;  //state variable index
static const int AVEL=1;	//state variable index


void main(int argc, char *argv)
{
	//we're using 32 samples, i.e., forward-simulated trajectories per animation frame. This corresponds to the N in the paper.
	const int nSamples=32;
	//physics simulation time step
	float timeStep=1.0f/30.0f;	

	//Allocate one simulation context for each sample, plus one additional "master" context
	initOde(nSamples+1);  
	setCurrentOdeContext(ALLTHREADS); 
	odeRandSetSeed(0);
	odeSetContactSoftCFM(0);
	odeWorldSetGravity(0, -9.81f, 0);

	//Build the model:
	//First create a capsule (capped cylinder) and orient it along the y axis (vertical). The capsule axis by default aligns with z axis.
	int geom=odeCreateCapsule(0,0.025f,1.0f);
	dQuaternion q;
	dQFromAxisAndAngle(q,1,0,0,PI*0.5f);
	odeGeomSetQuaternion(geom,q);
	//Create a body and attach it to the geom
	int body=odeBodyCreate();
	odeBodySetDynamic(body);
	//odeBodySetMass(body,1);
	odeGeomSetBody(geom,body);
	//Shift the capsule so that one end is at origin
	odeBodySetPosition(body,0,0.5f,0);
	//Create a hinge joint between the world and the capsule, with anchor at origin and z axis as the rotation axis
	int hinge=odeJointCreateHinge();
	odeJointAttach(hinge,0,body);
	odeJointSetHingeAnchor(hinge,0,0,0);
	odeJointSetHingeAxis(hinge,0,0,1);

	//We're done, now we should have nSamples+1 copies of a pendulum model. The last thing to do is to define the starting state for the master context (0).
	//We perturb the pendulum a bit and let it fall to a resting position. 
	setCurrentOdeContext(0); 
	dVector3 torque={0,0,1};
	odeBodyAddTorque(body,torque);
	const dReal *pos=odeBodyGetPosition(body);
	printf("Initial state: capsule y=%f, hinge angle=%f, avel=%f\n",pos[1],odeJointGetHingeAngle(hinge),odeJointGetHingeAngleRate(hinge));
	for (float i=0; i<10.0f/timeStep; i+=timeStep)
	{
		//apply some angular damping
		const dReal *avel=odeBodyGetAngularVel(body);
		odeBodySetAngularVel(body,avel[0]*0.9f,avel[1]*0.9f,avel[2]*0.9f);
		//step the simulation
		stepOde(timeStep,false);
	}
	pos=odeBodyGetPosition(body);
	printf("Initial state after perturbation: capsule y=%f, hinge angle=%f, avel=%f\n",pos[1],odeJointGetHingeAngle(hinge),odeJointGetHingeAngleRate(hinge));

	//Finally, save this state as the starting state
	saveOdeState(0);


	//initialize the optimizer
	ControlPBP pbp;
	int nTimeSteps=15;		//K in the paper, resulting in a 0.5s planning horizon
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
	float controlDiffStd=100.0f*C;	//sqrt(\sigma_{1}^2 C_u) in the pape. In effect, a "tolerance" for angular jerk minimization in this test
	float controlDiffDiffStd=100.0f*C; //sqrt(\sigma_{2}^2 C_u) in the paper. A large value to have no effect in this test.
	float mutationScale=0.25f;		//\sigma_m in the paper
	pbp.init(nSamples,nTimeSteps,nStateDimensions,nControlDimensions,&minControl,&maxControl,&controlMean,&controlStd,&controlDiffStd,&controlDiffDiffStd,mutationScale,NULL);

	//set further params: portion of "no prior" samples, resampling threshold, whether to use the backwards smoothing pass, and the regularization of the smoothing pass
	pbp.setParams(0.1f,0.5f,true,0.001f);  

	//run the algorithm for 90 steps (3 seconds)
	for (int n=0; n<90; n++)
	{
		//init all trajectories to the master state
		for (int i=0; i<nSamples; i++)
		{
			//activate the context for this sample
			setCurrentOdeContext(i+1); 
			//load the state from the master context
			restoreOdeState(0);
			//save the state to the sample context (needed later in resampling)
			saveOdeState(i+1,0);  
		}

		//signal the start of new C-PBP iteration
		setCurrentOdeContext(0); 
		restoreOdeState(0); 
		float angle=odeJointGetHingeAngle(hinge);
		float aVel=odeJointGetHingeAngleRate(hinge);
		float stateVector[2]={angle,aVel};
		pbp.startIteration(true,stateVector);

		//simulate forward 
		for (int k=0; k<nTimeSteps; k++)
		{
			//signal the start of a planning step
			pbp.startPlanningStep(k);
			//NOTE: for multithreaded operation, one would typically run each iteration of the following loop in a separate thread. 
			//The getControl(), getPreviousSampleIdx(), and updateResults() methods of the optimizer are thread-safe.
			//The physics simulation is also thread-safe as we have full copies of the simulated system for each thread/trajectory
			for (int i=0; i<nSamples; i++)
			{
				//get control from C-PBP
				float control;
				pbp.getControl(i,&control);

				//get the mapping from this to previous state (affected by resampling operations). This is the "history function" of the paper.
				int previousStateIdx=pbp.getPreviousSampleIdx(i);

				//simulate to get next state.
				setCurrentOdeContext(i+1);
				restoreOdeState(previousStateIdx+1); //continue the trajectory from where the previous forward simulation step ended 
				dVector3 torque={0,0,control};
				odeBodyAddTorque(body,torque); //apply the control
				stepOde(timeStep,true);

				//evaluate state cost
				float angle=odeJointGetHingeAngle(hinge);
				float aVel=odeJointGetHingeAngleRate(hinge);
				float cost=squared(angle*10.0f) + squared(aVel*1.0f);

				//Store the state feature vector and cost to C-PBP. 
				//Note that the stored state does not need to contain full simulation state. For example, only storing the angle works almost equally well here.
				//Selecting the state variables for the state feature vector is a balancing act. In principle one wants full state, but high dimensionality of the state
				//may make the results worse unless nSamples is increased to an infeasible value.
				float stateVector[2]={angle,aVel};
				pbp.updateResults(i,&control,stateVector,cost);
			}
			//save all states, will be used at next step (the restoreOdeState() call above)
			for (int i=0; i<nSamples; i++)
			{
				saveOdeState(i+1,i+1);
			}

			//signal the end of the planning step. this normalizes the state costs etc. for the next step
			pbp.endPlanningStep(k);
		}
		//signal the end of an iteration. this also executes the backwards smoothing pass
		pbp.endIteration();

		//deploy the best control found using the master context
		float control;
		pbp.getBestControl(0,&control);
		float cost=(float)pbp.getSquaredCost();
		setCurrentOdeContext(0);
		restoreOdeState(0);
		dVector3 torque={0,0,control};
		odeBodyAddTorque(body,torque);
		stepOde(timeStep,false);

		//save the resulting state as the starting state for the next iteration
		saveOdeState(0);	

		//print output, both angle and aVel should converge to 0, with some sampling noise remaining.
		angle=odeJointGetHingeAngle(hinge);
		aVel=odeJointGetHingeAngleRate(hinge);
		printf("Pendulum angle %1.3f, avel %1.3f, cost=%1.3f\n",angle,aVel,cost);
	} 
	printf("Done, press enter.");
	getchar();
}
