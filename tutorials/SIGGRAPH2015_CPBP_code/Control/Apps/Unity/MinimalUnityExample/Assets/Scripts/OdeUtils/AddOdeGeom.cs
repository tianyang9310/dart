using UnityEngine;
using System.Collections;


//Finds a sphere or box collider and creates the corresponding ODE geom
public class AddOdeGeom : MonoBehaviour {
    public int geomId = -1;
    public Vector3 posShift = Vector3.zero;
    public bool debugVisualize = false;
    public bool dynamic = false;
    public float density = 1.0f;
    // Use this for initialization
	void Start () {
	    
	}
	
    //The object managing physics will call this at the correct time
    public void initialize()
    {
        if (dynamic)
        {
            if (GetComponent<SphereCollider>() != null)
            {
                gameObject.tag = "Ball";
                SphereCollider collider = GetComponent<SphereCollider>();
                float radius = collider.radius * Mathf.Max(collider.transform.lossyScale.x, Mathf.Max(collider.transform.lossyScale.y, collider.transform.lossyScale.z));
                var body = gameObject.AddComponent<OdeBody>();
                var geom = gameObject.AddComponent<OdeGeomSphere>();
                geom.debug = false;
                UnityOde.odeGeomSphereSetRadius(geom.GeomId, radius);
                GameObject.Destroy(geom.debugSphere);//.transform.localScale = new Vector3(radius, radius, radius)*2.0f/transform.localScale.x;
                UnityOde.odeMassSetSphere(body.BodyId, density, radius);
                body.position = transform.position;
                body.updateTransfromFromPhysics();
            }
        }
        else
        {
            if (GetComponent<SphereCollider>() != null)
            {
                SphereCollider collider = GetComponent<SphereCollider>();
                float radius = collider.radius * Mathf.Max(collider.transform.lossyScale.x, Mathf.Max(collider.transform.lossyScale.y, collider.transform.lossyScale.z));
                Vector3 pos = transform.TransformPoint(collider.center);

                geomId = UnityOde.odeCreateSphere(radius);
                UnityOde.odeGeomSetPosition(geomId, pos.x, pos.y, pos.z);
                if (debugVisualize)
                {
                    GameObject debugSphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    debugSphere.transform.localScale = new Vector3(radius, radius, radius) * 2.0f;
                    debugSphere.transform.position = pos;
                }
            }
            else if (GetComponent<BoxCollider>() != null)
            {
                BoxCollider collider = GetComponent<BoxCollider>();
                Vector3 size = new Vector3(collider.size.x * transform.lossyScale.x, collider.size.y * transform.lossyScale.y, collider.size.z * transform.lossyScale.z);
                Vector3 pos = transform.TransformPoint(collider.center);
                geomId = UnityOde.odeCreateBox(size.x, size.y, size.z);
                UnityOde.odeGeomSetPosition(geomId, pos.x, pos.y, pos.z);
                UnityOde.odeGeomSetQuaternion(geomId, transform.rotation);
                if (debugVisualize)
                {
                    GameObject debugBox = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    debugBox.name = "dbgBox_" + gameObject.name;
                    debugBox.transform.position = pos;
                    debugBox.transform.rotation = transform.rotation;
                    debugBox.transform.localScale = size;
                }

            }
        }
    }

	// Update is called once per frame
	void Update () {
	}
}
