// Terminology:
// Wrapper class - Generated C++ X_wrap.cpp file that contains the exported unmanaged API.
// Intermediate class - Generated C# P/Invoke file (ie. the mapped managed API of the wrapper class)
// Proxy class - Generated C# class that is called by the C# code and handles calling the managed API and data marshaling.

%include "typemaps.i"

%module UnityOptimizationPlugin
%{
/* this code will be inserted to the generated UnityOptimizationPlugin_wrap.cpp */
#include "ControlPBP.h"

using namespace AaltoGames;
%}

 // Add some helper functions to the C# proxy class
 // This is needed because automatic struct marshaling in C# uses struct constructors for initialization instead of
 // directly marshaling struct fields. Unity Quaternion class has its fields in the order w, x, y, z but the 
 // constructor parameters are ordered (x, y, z, w) while the data in ODE memory is actually laid out as (w, x, y, z).
 // Because of this we need to manually marshal the data.
 // NOTE: currently only works with single-precision, ie. ODE compiled with double precision won't work.
 %pragma(csharp) modulecode=%{
  private static void MarshalHelperWrite(ref IntPtr ptr, ref UnityEngine.Quaternion q)
  {
	Marshal.WriteInt32(ptr, Marshal.SizeOf(typeof(float)) * 0, BitConverter.ToInt32(BitConverter.GetBytes(q[3]), 0));
    Marshal.WriteInt32(ptr, Marshal.SizeOf(typeof(float)) * 1, BitConverter.ToInt32(BitConverter.GetBytes(q[0]), 0)); 
	Marshal.WriteInt32(ptr, Marshal.SizeOf(typeof(float)) * 2, BitConverter.ToInt32(BitConverter.GetBytes(q[1]), 0));
    Marshal.WriteInt32(ptr, Marshal.SizeOf(typeof(float)) * 3, BitConverter.ToInt32(BitConverter.GetBytes(q[2]), 0));
  }

  private static void MarshalHelperRead(ref IntPtr ptr, out UnityEngine.Quaternion q)
  {
    q = new UnityEngine.Quaternion(BitConverter.ToSingle(BitConverter.GetBytes(Marshal.ReadInt32(ptr, Marshal.SizeOf(typeof(float)) * 1)), 0),
	                               BitConverter.ToSingle(BitConverter.GetBytes(Marshal.ReadInt32(ptr, Marshal.SizeOf(typeof(float)) * 2)), 0),
								   BitConverter.ToSingle(BitConverter.GetBytes(Marshal.ReadInt32(ptr, Marshal.SizeOf(typeof(float)) * 3)), 0),
								   BitConverter.ToSingle(BitConverter.GetBytes(Marshal.ReadInt32(ptr, Marshal.SizeOf(typeof(float)) * 0)), 0));
  }
%}

// Typemaps for vector types (ie. vector, quaternion). Handles return values and function input/output values
%define TYPEMAP_VECTOR(CTYPE, IMTYPE, CSTYPE)

// These typemaps are generated for the C++ wrapper class
%typemap(ctype, out="/* ctype out */ CTYPE")    CTYPE "/* ctype */ CTYPE"
%typemap(in)                                    CTYPE "/* in */ $1 = $input;"
%typemap(out, null="/* null */ NULL")           CTYPE "/* out */ $result = $1;"
%typemap(argout)                                CTYPE "/* argout */"

// These typemaps are for the intermediate class
%typemap(imtype, out="/* imtype out */ IMTYPE") CTYPE "/* imtype */ IMTYPE"

// These typemaps are for the proxy class
%typemap(cstype, out="/* cstype out */ CSTYPE") CTYPE "/* cstype */ CSTYPE"
%typemap(cstype, out="/* cstype out */ CSTYPE") CTYPE OUTPUT "/* cstype output */ out CSTYPE"
%typemap(csin,
         pre= "    /* csin pre */ IMTYPE ptr_$csinput = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(CSTYPE)));\n"
	          "    Marshal.StructureToPtr($csinput, ptr_$csinput, false);",
         post="      /* csin post */ Marshal.FreeHGlobal(ptr_$csinput);")
		                                        CTYPE "/* csin */ ptr_$csinput"
%typemap(csin,
         pre= "    /* csin pre output */ IMTYPE ptr_$csinput = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(CSTYPE)));",
		 post="      /* csin post output */ $csinput = (CSTYPE)Marshal.PtrToStructure(ptr_$csinput, typeof(CSTYPE));\n"
		      "      Marshal.FreeHGlobal(ptr_$csinput);")
		                                        CTYPE OUTPUT "/* csin */ ptr_$csinput"
%typemap(csout, excode=SWIGEXCODE)              CTYPE
  {
    /* csout */ IMTYPE ret = $imcall;$excode
	return (CSTYPE)Marshal.PtrToStructure(ret, typeof(CSTYPE));
  }
   
%enddef

// Quaternions need some specific 'hacks' since ODE expects them to be [w, x, y, z] in memory and in Unity they are [x, y, z, w]
%define TYPEMAP_QUAT_FIX(CTYPE, IMTYPE, CSTYPE)

%typemap(csin,
         pre= "    /* csin pre */ IMTYPE ptr_$csinput = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(CSTYPE)));\n"
		      "    MarshalHelperWrite(ref ptr_$csinput, ref $csinput);",
         post="      /* csin post */ Marshal.FreeHGlobal(ptr_$csinput);")
		                                        CTYPE "/* csin */ ptr_$csinput"
%typemap(csin,
         pre= "    /* csin pre output */ IMTYPE ptr_$csinput = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(CSTYPE)));",
		 post="      /* csin post output */ MarshalHelperRead(ref ptr_$csinput, out $csinput);\n"
		      "      Marshal.FreeHGlobal(ptr_$csinput);")
		                                        CTYPE OUTPUT "/* csin */ ptr_$csinput"
%typemap(csout, excode=SWIGEXCODE)              CTYPE
  {
    /* csout */ IMTYPE ptr = $imcall;$excode
	CSTYPE ret;
	MarshalHelperRead(ref ptr, out ret);
	return ret;
  }

%enddef

// Macros for defining input and output function argument types. These can be used to create different typemaps
// for types that are labeled OUTPUT or INOUT. They also automatically add 'ref' and 'out' keywords to the
// function parameters in the C# intermediate and proxy classes
%define %TypeRefParam(TYPE)	
    %apply TYPE& INOUT { TYPE& };
%enddef

%define %TypeOutParam(TYPE)
    %apply TYPE& OUTPUT { TYPE& };
%enddef

// Define our vector types using the typemap macro
TYPEMAP_VECTOR(OdeVector, IntPtr, UnityEngine.Vector3)
TYPEMAP_VECTOR(ConstOdeVector, IntPtr, UnityEngine.Vector3)
TYPEMAP_VECTOR(OdeQuaternion, IntPtr, UnityEngine.Quaternion)
TYPEMAP_VECTOR(ConstOdeQuaternion, IntPtr, UnityEngine.Quaternion)

// Use this macro to override some typemaps for quaternions
TYPEMAP_QUAT_FIX(OdeQuaternion, IntPtr, UnityEngine.Quaternion)
TYPEMAP_QUAT_FIX(ConstOdeQuaternion, IntPtr, UnityEngine.Quaternion)

// Define vector and quaternion function parameters as OUTPUT if their name is 'result' or starts with 'out_'
%apply OdeVector OUTPUT { OdeVector result }
%apply OdeVector OUTPUT { OdeVector out_pos }
%apply OdeVector OUTPUT { OdeVector out_normal }
%apply OdeVector OUTPUT { OdeVector out_vel }
%apply OdeVector OUTPUT { OdeVector out }
%apply OdeQuaternion OUTPUT { OdeQuaternion result }

// Define references to floats and ints as OUTPUT parameters
%TypeOutParam(float)
%TypeOutParam(int)

/* Parse the files to generate wrappers */
%include "..\..\Include\ControlPBP.h"
