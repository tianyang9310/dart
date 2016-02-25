using System;
using UnityEngine;
using System.Runtime.InteropServices;

class OdeWorld : MonoBehaviour
{
    public static bool initialized = false;
	public int nThreads=1;
	public bool autoStep=true;	//set this to false if you want to control stepping from some other object
    // TODO: currently this is set to start before other scripts, shouldn't be like that
    void Awake()
    {
        // TODO: these are currently not used
        int spaceId = -1;

        // TODO: move these elsewhere
        OdeWorld.initialized = UnityOde.initOde(nThreads);
        UnityOde.odeCreatePlane(spaceId, 0, 1, 0, 0);
    }

    void Start() 
    {
        /*var capsule1 = GameObject.Find("OdeCapsule").GetComponent<OdeBody>();
        var capsule2 = GameObject.Find("OdeCapsule2").GetComponent<OdeBody>();

        var joint = capsule2.gameObject.AddComponent<OdeJointBallAndSocket>();
        joint.connectedBody = capsule1;

        joint.anchor = capsule2.transform.position;
        joint.cfm = 0;*/
    }

    void FixedUpdate()
    {
        UnityOde.stepOde(Time.fixedDeltaTime, true);
    }

    void OnApplicationQuit()
    {
        UnityOde.uninitOde();
        OdeWorld.initialized = false;
    }
}
