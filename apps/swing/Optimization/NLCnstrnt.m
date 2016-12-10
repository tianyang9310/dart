function [c, ceq] = NLCnstrnt(x);  
global g;
global r;
c = [];     
ceq = 2*g*cos(x(1))-r*x(2)^2;
end
