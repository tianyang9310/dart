# Pre

Compiling instruction:
For Mac OS
```
cd build
cmake ..
make
```

# Part 1:

Red ball is using analytic solution. The second ball is using Explicit Euler method, the third one is using Mid-Point method and the last one is using RK4 method.

#### Extra Credit:

* The third integration method is RK4 method. It is accurate as Mid-Point and analytic solution

* At the very beginning, the user can input initial vertical velocity in the terminal

# Part 2:

The system is evolving including constrain force, such that the first bead stays on the wire and the distance between two beads stays the same.

When applying constrain force to the system, feedback terms need to be considered to guarantee that the constraints are satisfied in the long run.

The system is in essentially harmonically oscillating. All explicit integrator method may not stabilize the simulation. It turns out that Implicit Euler can stabilize the system.  

#### Extra Credit:

* The user could change the integrator at any time.   
  `f` for Forward Euler method  
  `m` for Mid-Point method  
  `k` for Forward Euler method  
  `i` for Implicit Euler method (default)

* The user could use mouse click to select the closest particle and use mouse drag to apply force on that particle. The magnitude and the direction of the force is consistent with the mouse action.
