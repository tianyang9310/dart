## Compilation instruction
### For Mac OS:  
-------------------------
cmake ..  
make

### Requirements:
-------------------------
* Using loop to formulate IK problem for arbitrary skeleton and constraints
* Solving IK in real time
* Solving multiple constraints at the same time. Once a constraint is added, it will remain active until the user remove it.

### Extra Credits:
------------------------
* Press `o` to toggle objective which tries to match the initial pose.
* Press `j` to toggle joint limit (_only knee joint limits are enforced_). When joint limit is on, knees never bend strangely.
* Press `l` to toggle left and right hand constraint. The weight of this constraint is set as 10.0, such that it would be respected even if left or right hand has its own target constraint.
* Press `r` to reset biped to initial position.
* Adaptively decaying learning rate alpha.
* For better visualization, when one marker is activated, it becomes blue and its target appears as green. 
