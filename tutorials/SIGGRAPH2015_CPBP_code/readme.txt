This is the initial (1.0) code release for the following paper: 

Hämäläinen, P., Rajamäki, J., Liu, C.K., Online Control of Simulated Humanoids Using Particle Belief Propagation, Proc. SIGGRAPH 2015

Control/Apps/Unity contains test projects compatible with Unity 5. 

Control/Src contains the implementation of C-PBP. This is compiled to an Unity dll using the Control/Apps/UnityOptimizationPlugin project. If you make changes to C-PBP, the Unity apps don't import the changes automatically - instead, you need to select "Import plugins" from the AaltoGames menu.

For now, Control/Apps/Unity is lacking the biped project, which needs some additional clean-up. The MinimalUnityExample project should however give one a good basis for experimenting. In the project, a simple cost function states that a ball should be at the origin, and C-PBP controls the torques of another ball. This makes shepherding behavior emerge: one ball pushes the other to the origin. Pressing enter allows one to give random impulses to the shepherded ball.


