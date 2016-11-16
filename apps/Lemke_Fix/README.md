1. 1d-trivial problem

2. for 2-D problem, Lemke solution is not correct, but it will return error type

```
There are 2 contact points
-----------------------------
---Solve LCP via Lemke-------
Eigen::MatrixXd A is now
  3.12 0.1877
0.1877  3.254
Eigen::VectorXd b is now
 -0.00662
-0.006711
Using Lemke to solve the LCP problem
error: 3
z 0.002005        0
invalid Lemke solution
-----------------------------

After solve:
A: 
3.12 0.1877 -4.02 -0 
0.1877 3.254 6.568e-317 6.565e-317 
b: 0.00662 0.006711 
w: 0 0 
x: 0.002005 0.001947 
Ax   : 0.00662 0.006711 
b + w: 0.00662 0.006711 
```
Both above will have solution, but Lemke will return some incorrect solution with error type


3. 2 contact points have 3 by 3 matrix, 3 contact points have 2 by 2 matrix
The reason is that the pipeline of how dart simulates. Everytime I get the numConstraints of the system, it is actually have another constraint-less simulation

4. some nan result from error type 0

```
-----------------------------
---Solve LCP via Lemke-------
Eigen::MatrixXd A is now
4.001    -2
   -2 3.999
Eigen::VectorXd b is now
-0.009819
-0.009798
Using Lemke to solve the LCP problem
error: 0
z -nan    0
-----------------------------

After solve:
A: 
4.001 -2 0.06683 -0.0001976 
-2 3.999 1.001 -2 
b: 0.009819 0.009798 
w: 0 0 
x: 0.004905 0.004903 
Ax   : 0.009819 0.009798 
b + w: 0.009819 0.009798 
```
