using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Runtime.InteropServices;
using System.Threading;


public class BallControl : MonoBehaviour {
    //define the problem dimensionality
    const int nControls = 2;  //two torques
    const int nStates = 12; //2*3 positions and velocities

    //for debug
    public double stateCostDisplay = 0;
    public double trajectoryCostDisplay = 0;
    //set to false to enable more flexible debugging
    public bool useThreads = false;
    //if true, don't sample, simply use the learned policy
    public bool useLearned = false;
    //simulation timestep
    public float timeStep = 1.0f / 30.0f;
    //prior stdev for torques
    public float torqueSd = 1.0f;
    //resampling threshold
    public float resamplingThreshold = 0.5f;
    //trajectory mutation sd (\sigma_m in the paper)
    public float mutationSd = 0.1f;
    //sd or tolerance for the target distance from controlled object
    public float targetDistSd = 0.1f;
    //sd or tolerance for the target distance from origin
    public float targetPosSd = 1.0f;
    //sd or tolerance for the target velocity (the controlled ball attempts to both hit the target and make it slow down)
    public float targetVelSd = 0.1f;
    //sd or tolerance for the controlled sphere velocity
    public float controlledVelSd = 0.1f;
    //number of trajectories simulated in parallel
    public int nTrajectories = 32;
    //planning horizon length
    public float planningHorizonSeconds = 1.0f;
    //number of forward prediction steps in the graph, computed from timeStep and planningHorizonSeconds
    int nSteps;
    [HideInInspector]
    public int maxPhysicsContexts = 260; //needed for batch testing - right now one can't reallocate more contexts once allocated
    //Optimizer instance
    ControlPBP opt;
    //Optimizer initialization params and helpers
    PinnedArray<float> minValues, maxValues;
    PinnedArray<float> controlPriorStd;
    PinnedArray<float> controlMean;
    PinnedArray<float> controlDiffPriorStd;
    PinnedArray<float> controlDiffDiffPriorStd;

    //Something to hold the body id of the controlled ODE objects
    int controlledBody, targetBody, targetGeom, controlledGeom;
    float ballRadius; 

    //A class and array to hold things for each thread
    class ThreadContext
    {
        public PinnedArray<float> state, control;
        public int physicsContext;
    }
    ThreadContext[] contexts;
    ThreadContext masterContext;
  

    //events for thread signaling
    ManualResetEvent[] threadEvents;

    public double bestTrajectoryProbability = 0;


	// Use this for initialization
	void Start () {

        //create thread contexts
        contexts = new ThreadContext[nTrajectories + 1]; //+1 because of one master context
        for (int i = 0; i <= nTrajectories; i++)
        {
            contexts[i] = new ThreadContext();
            contexts[i].state = new PinnedArray<float>(nStates);
            contexts[i].control = new PinnedArray<float>(nControls);
            contexts[i].physicsContext = i;
        }
        masterContext = contexts[nTrajectories];   

        //Sync Unity's fixed update with our physics
        Time.fixedDeltaTime = timeStep;

        //Set up ODE world
        UnityOde.initOde(nTrajectories+1);  //+1 because of one master context 
        UnityOde.odeRandSetSeed((uint)System.DateTime.Now.Ticks);
        UnityOde.setCurrentOdeContext(UnityOde.ALLTHREADS);
        UnityOde.odeSetContactSoftCFM(0);
        UnityOde.odeWorldSetGravity(0, -9.81f, 0);

        //create ground plane
        int groundPlaneBodyId = UnityOde.odeCreatePlane(0, 0, 1, 0, 0);
        Debug.Log("Ground plane: " + groundPlaneBodyId);
        UnityOde.odeGeomSetCategoryBits(groundPlaneBodyId, 0xf0000000);
        UnityOde.odeGeomSetCollideBits(groundPlaneBodyId, 0xffffffff);

        //create other scene physics objects
        AddOdeGeom[] aog = GameObject.FindObjectsOfType<AddOdeGeom>();
        foreach (AddOdeGeom a in aog)
        {
            a.initialize();
        }

        //query the body id (integer) of the controlled object, as GetComponent and other unity methods
        //can't be called later from the optimizer background threads
        controlledBody = (GameObject.Find("Sphere") as GameObject).GetComponent<OdeGeomSphere>().Body.BodyId;
        targetBody = (GameObject.Find("Target") as GameObject).GetComponent<OdeGeomSphere>().Body.BodyId;
        controlledGeom = (GameObject.Find("Sphere") as GameObject).GetComponent<OdeGeomSphere>().GeomId;
        targetGeom = (GameObject.Find("Target") as GameObject).GetComponent<OdeGeomSphere>().GeomId;

        

        //set some initial velocity for our target
        UnityOde.setCurrentOdeContext(masterContext.physicsContext);
        UnityOde.odeBodySetLinearVel(targetBody, 2, 0, 0);
        UnityOde.saveOdeState(); //remember to save the state, because the state is reset in subsequent optimization simulations

        //set up the optimizer.
        //first initialize the various float vectors needed
        minValues = new PinnedArray<float>(nControls);
        maxValues = new PinnedArray<float>(nControls);
        controlMean = new PinnedArray<float>(nControls);
        controlPriorStd = new PinnedArray<float>(nControls);
        controlDiffPriorStd = new PinnedArray<float>(nControls);
        controlDiffDiffPriorStd = new PinnedArray<float>(nControls);

        for (int i = 0; i < nControls; i++)
        {
            minValues[i] = -1000;  //not caring about hard limits here, control limited only by the sd:s
            maxValues[i] = 1000;
            controlMean[i] = 0;
            controlPriorStd[i] = torqueSd;
            controlDiffPriorStd[i] = torqueSd * 100; //large number, i.e., no effect
            controlDiffDiffPriorStd[i] = torqueSd * 100; //large number, i.e., no effect
        }


        //create and init the optimizer instance
        opt = new ControlPBP();
        nSteps=(int)(planningHorizonSeconds/timeStep);
        opt.init(nTrajectories, nSteps, nStates, nControls, minValues, maxValues, controlMean, controlPriorStd, controlDiffPriorStd, controlDiffDiffPriorStd, mutationSd, null);

        //initialize thread events
        threadEvents = new ManualResetEvent[nTrajectories];
        for (int i = 0; i < threadEvents.Length; ++i)
        {
            threadEvents[i] = new ManualResetEvent(false);
        }

    }
    void updateStateVector(PinnedArray<float> state)
    {
        //the position of the controlled object (equal to the relative position of goal, which is at origin)
        Vector3 pos=UnityOde.odeBodyGetPosition(controlledBody);
        state[0] = pos.x;
        state[1] = pos.y;
        state[2] = pos.z;
        //the velocity of the controlled object (equal to the relative position of goal, which is at origin)
        Vector3 vel = UnityOde.odeBodyGetLinearVel(controlledBody);
        state[3] = vel.x;
        state[4] = vel.y;
        state[5] = vel.z;
        //relative position of the target object (equal to the relative position of goal, which is at origin)
        pos -= UnityOde.odeBodyGetPosition(targetBody);
        state[6] = pos.x;
        state[7] = pos.y;
        state[8] = pos.z;
        //the velocity of the target object 
        vel = UnityOde.odeBodyGetLinearVel(targetBody);
        state[9] = vel.x;
        state[10] = vel.y;
        state[11] = vel.z;
    }
    void FixedUpdate()
    {
        //update optimizer parameters
        opt.setParams(0.25, resamplingThreshold, true, 0.001f);
        //Active the "master context", i.e., 0
        UnityOde.setCurrentOdeContext(masterContext.physicsContext);  
        if (Input.GetKey(KeyCode.Return))
            UnityOde.odeBodySetLinearVel(targetBody, UnityEngine.Random.Range(-2.0f, 2.0f), UnityEngine.Random.Range(-2.0f, 2.0f), 0);

        //Update the current state and pass it to the optimizer
        updateStateVector(contexts[masterContext.physicsContext].state);
        //optimize
        opt.startIteration(true, contexts[masterContext.physicsContext].state);
        //loop over forward prediction steps
        for (int step = 0; step < nSteps; step++)
        {
            //signal the start of a planning step (forward prediction step)
            opt.startPlanningStep(step);
            // Reset events 
            for (int i = 0; i < threadEvents.Length; ++i) { threadEvents[i].Reset(); }

            //loop over trajectories, initialize physics starting states
            for (int trajectory = 0; trajectory < nTrajectories; trajectory++)
            {
                if (step == 0)
                    //save the physics state: at first step, the master context is copied to every other context
                    UnityOde.saveOdeState(contexts[trajectory].physicsContext, masterContext.physicsContext);
                else
                    //at others than the first step, just save each context so that the resampling can branch the paths
                    UnityOde.saveOdeState(contexts[trajectory].physicsContext, contexts[trajectory].physicsContext);
            }

            //loop over trajectories, get control from optimizer and simulate one step forward
            for (int trajectory = 0; trajectory < nTrajectories; trajectory++)
            {
                if (useThreads)
                    ThreadPool.QueueUserWorkItem(runControlStep, trajectory);
                else
                {
                    runControlStep(trajectory);
                }
            }
            //wait for all the threads, no background processing should be running when calling endPlanningStep
            WaitHandle.WaitAll(threadEvents);
            opt.endPlanningStep(step);
        }
        //finalize things, this includes the Gaussian backwards smoothing as well
        opt.endIteration();
        //get the best control, and deploy it to the master context
        UnityOde.setCurrentOdeContext(masterContext.physicsContext);
        updateStateVector(masterContext.state);
        opt.getBestControl(0, contexts[masterContext.physicsContext].control);
        bestTrajectoryProbability = Math.Exp(-opt.getSquaredCost() * 0.001f);
        //some debug output
        trajectoryCostDisplay = opt.getSquaredCost();
        stateCostDisplay = computeStateCost();
        applyControl(contexts[masterContext.physicsContext].control);
        UnityOde.stepOde(timeStep, false);
        UnityOde.saveOdeState();

    }

    void runControlStep(object o)
    {
        int trajectory = (int)o;
        //query the history function
        int previousStateIdx = opt.getPreviousSampleIdx(trajectory);
        //start simulating from the correct state
        UnityOde.setCurrentOdeContext(contexts[trajectory].physicsContext);
        ThreadContext t = contexts[contexts[trajectory].physicsContext];
        UnityOde.restoreOdeState(contexts[previousStateIdx].physicsContext);
        //remember the starting position for debug drawing
        Vector3 lineStart = UnityOde.odeBodyGetPosition(controlledBody);

        //obtain a control vector
        opt.getControl(trajectory, t.control);
        //apply the control to the simulation (torques around the x and z axes)
        applyControl(t.control);
        //store the current best trajectory to learn a policy
        if (trajectory == 0 && bestTrajectoryProbability>0) 
        {
            updateStateVector(t.state);
        }
        //run the simulation. Note that the biped example works a bit differently as it runs several physics steps per control step to save CPU
        UnityOde.stepOde(timeStep, false);
        //compute cost
        float cost = computeStateCost();
        //save the results to the optimizer
        updateStateVector(t.state);
        opt.updateResults(trajectory, t.control, t.state, cost);

        //draw debug trajectory (if not threaded)
        if (!useThreads)
        {
            Vector3 lineEnd = UnityOde.odeBodyGetPosition(controlledBody);
            Color color = Color.gray;
            if (trajectory == 0)
                color = Color.green; //previous best
            else if (trajectory == 1)
                color = Color.yellow; //gaussian backwards smoothed
            Debug.DrawLine(lineStart, lineEnd, color);
        }
        //signal that we're done
        threadEvents[trajectory].Set();
    }

    private float computeStateCost()
    {
        //minimize distance between spheres (minus the sum of the radiuses)
        Vector3 pos = UnityOde.odeBodyGetPosition(controlledBody);
        Vector3 targetPos = UnityOde.odeBodyGetPosition(targetBody);
        float minDistance=UnityOde.odeGeomSphereGetRadius(controlledGeom)+UnityOde.odeGeomSphereGetRadius(targetGeom);
        float dist=(pos - targetPos).magnitude-minDistance;
        float cost = dist * dist / (targetDistSd * targetDistSd);
        //minimize the target sphere velocity
        cost += UnityOde.odeBodyGetLinearVel(targetBody).sqrMagnitude/(targetVelSd*targetVelSd);
        //minimize our own velocity
        cost += UnityOde.odeBodyGetLinearVel(controlledBody).sqrMagnitude / (controlledVelSd * controlledVelSd);
        //minimize the target sphere deviation from origin
        cost += targetPos.sqrMagnitude/(targetPosSd*targetPosSd);
        return cost;
    }

    private void applyControl(PinnedArray<float> control)
    {
         UnityOde.odeBodyAddTorque(controlledBody,new Vector3(control[0],0,control[1]));
    }

	// Update is called once per frame
	void Update () {
	    
	}
    void OnApplicationQuit()
    {
        WaitHandle.WaitAll(threadEvents);
        UnityOde.uninitOde();

        // The following voodoo is essential to avoid memory leaks in unity editor
#if UNITY_EDITOR
        UnityEditor.EditorUtility.UnloadUnusedAssetsImmediate();
#endif
        opt = null;
        GC.Collect();

    }
}
