clc,clear; close all
load CropA.mat

%? copy to make symmetric 0 and 2.21e-16
% grap data from ODE to build A of Lemke
% first diagnoal block
Agent = -100*ones(size(A_ODE));
numcontacts = 4;
% normal to normal
Agent(1:4,1:4) = [A_ODE(1,1),A_ODE(1,10),A_ODE(1,19),A_ODE(1,28);
    A_ODE(10,1),A_ODE(10,10),A_ODE(10,19),A_ODE(10,28);
    A_ODE(19,1),A_ODE(19,10),A_ODE(19,19),A_ODE(19,28);
    A_ODE(28,1),A_ODE(28,10),A_ODE(28,19),A_ODE(28,28)];

% ct1
Agent(5:12,5:12) = A_ODE(2:9,2:9);
% ct2
Agent(13:20,13:20) = A_ODE(11:18,11:18);
% ct3
Agent(21:28,21:28) = A_ODE(20:27,20:27);
% ct4
Agent(29:36,29:36) = A_ODE(29:36,29:36);

% off-diagonal
% normal - ct1
Agent(1:4,5:12) = A_ODE(1:9:36,2:9); 
% normal - ct2
Agent(1:4,13:20) = A_ODE(1:9:36,11:18); 
% normal - ct3
Agent(1:4,21:28) = A_ODE(1:9:36,20:27); 
% normal - ct4
Agent(1:4,29:36) = A_ODE(1:9:36,29:36); 

% ct1 - ct2
Agent(5:12,13:20) = A_ODE(2:9,11:18);
% ct1 - ct3
Agent(5:12,21:28) = A_ODE(2:9,20:27);
% ct1 - ct4
Agent(5:12,29:36) = A_ODE(2:9,29:36);

% ct2 - ct3
Agent(13:20,21:28) = A_ODE(11:18,20:27);
% ct2 - ct4
Agent(13:20,29:36) = A_ODE(11:18,29:36);

% ct3 - ct4
Agent(21:28,29:36) = A_ODE(20:27,29:36);

tmp=triu(Agent)';
Agent(find(Agent==-100)) = tmp(find(Agent==-100));