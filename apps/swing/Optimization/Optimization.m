global h;
global r;
global g;
h = 2.0;
r = 0.9278;
g = 9.81;
X0 = [0.2716;  6.1483];  
A = [];  
B = [];  
Aeq = [];  
Beq = [];  
LB = [0, compute_theta_dot(pi/4)];  
UB = [pi/4,compute_theta_dot(0)];  
[x, fval] = fmincon('MyObjective', X0, A, B, Aeq, Beq, LB, UB, 'NLCnstrnt') 
d1 = r*sin(x(1))
d1_dot = r*x(2)*cos(x(1))
