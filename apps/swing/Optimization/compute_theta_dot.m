function theta_dot = compute_theta_dot(theta)
global g;
global r;
theta_dot = sqrt(2*g*cos(theta)/r);
end
