function [ y ] = MyObjective( x )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
global h;
global r;
global g;

y = -(r^2*x(2)^2*sin(x(1))*cos(x(1)) + r*x(2)*cos(x(1))*sqrt(2*g*h-r^2*x(2)^2)*(cos(x(1)))^2)/g;

end

