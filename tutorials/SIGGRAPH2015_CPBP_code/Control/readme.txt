This folder contains control optimization methods and tools.

Note: the code is currently not polished or properly commented, except for InvPendulumTest.cpp, which should provide a clear and simple example of how to use the Control Particle Belief Propagation algorithm. 

The code should compile on various platforms, but has only been tested on Windows. Apps/InvPendulumTest.vxproj is a minimal Visual Studio test project with dependencies only to the Eigen. The projects should be convertable to other platforms, see: http://stackoverflow.com/questions/6649606/vcxproj-to-cmake

Apps/Unity/MinimalUnityExample is a project that uses C-PBP to control a ball to shepherd another ball to origin, as an example of complex behavior emerging from a simple cost functin. To run, 1) build the UnityOptimizationPlugin MSVC project, 2) load the Unity project, 3) select AaltoGames/import dll from the menu. Whenever you rebuild the MSVC part, you must restart Unity and do 3) again.




Dependencies to 3rd party libraries:

Eigen  (The projects in Apps folder are configured to look for Eigen at ..\..\ThirdParty\eigen)
ODE (some projects assume ODE to be located in ..\..\ThirdParty\Ode-0.12)

For convenience, the 3rd party libraries are also provided as a .zip in the root folder. The version of ode contains some causality patches (see Hämäläinen et al., "Online Motion Synthesis Using Sequential Monte Carlo", proc. SIGGRAPH 2014 for discussion on the simulation causality)
