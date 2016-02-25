using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.IO;


#if UNITY_EDITOR
using UnityEditor;
#endif

public class ImportUnityRRT : MonoBehaviour {
    //Editor stuff
#if UNITY_EDITOR
	// Helper for copying the built tracker dll to the plugins folder
	[MenuItem("AaltoGames/Import DLLs")]
#endif
	static void importDlls()
	{
        importDll("UnityOde");
        importDll("UnityOptimizationPlugin");
        //delete the duplicate SWIGTYPE_p_float, already exists as part of the ODE wrapper
        File.Delete("Assets/Scripts/UnityOptimizationPlugin/SWIGTYPE_p_float.cs");
    }
    static void importDll(string dllName)
    {
        //copy the dll if it has been built
        if (!tryCopy("..\\..\\"+dllName+"\\dll\\"+dllName+".dll", "Assets/Plugins/"+dllName+".dll"))
        {
            Debug.LogError("Cannot copy the "+dllName + " dll, check that you've built it!");
            return;
        }
        //copy debug info if it exists
        tryCopy("..\\..\\"+dllName+"\\dll\\"+dllName+".pdb", "Assets/Plugins/"+dllName+".pdb");

        //copy all .cs files
        DirectoryInfo src = new DirectoryInfo("..\\..\\"+dllName+"\\");
        string dstDir="Assets/Scripts/"+dllName;
        if (!Directory.Exists(dstDir))
            Directory.CreateDirectory(dstDir);
        DirectoryInfo dst = new DirectoryInfo(dstDir);
        CopyFiles(src, dst, true, "*.cs");
        Debug.Log("Copied " + dllName+".dll and its .cs wrapper files");
    }

    static bool tryCopy(string dllName, string dstName)
	{
		if (File.Exists(dllName))
		{
			File.Copy(dllName,dstName,true);
			return true;
		}
		return false;
	}
    static void CopyFiles(DirectoryInfo source,
                          DirectoryInfo destination,
                          bool overwrite,
                          string searchPattern)
    {
        FileInfo[] files = source.GetFiles(searchPattern);

        //this section is what's really important for your application.
        foreach (FileInfo file in files)
        {
            file.CopyTo(destination.FullName + "\\" + file.Name, overwrite);
        }
    }
    // Use this for initialization
	void Start () {
	
	}
	
	// Update is called once per frame
	void Update () {
	
	}
}
