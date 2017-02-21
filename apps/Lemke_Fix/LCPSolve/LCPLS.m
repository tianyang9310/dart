function [z,time]=LCPLS(A,b,value)
n = length(b);

% Trivial solution exists
if all(b>=0)
    z=zeros(n,1); return;
end

% Initializations
% (note: memory allocation for all variables to avoid later reallocation)
z = zeros(2*n,1);
x=b;
bas=zeros(n,1);
nonbas=zeros(n,1);

t = 2*n+1; % Artificial variable

ub_index = value2ub_index(value);
z0 = zeros(n,1);
z0(ub_index) = 1;
% Determine initial basis
if isempty(z0)
    bas=[];
    nonbas=(1:n)';
else
    bas=find(z0>0);
    nonbas=find(z0<=0);
end

B = -eye(n);

B=[A(:,bas) B(:,nonbas)];

%   x=-(B\b);

% x = -pinv(B)*b;
% options = optimoptions('lsqlin','Display', 'off');
% options = optimoptions('Display', 'off');
% x = lsqlin(B,-b,[],[],[],[],zeros(n,1),[],[],options);

% x = lsqr(B,-b);
% options = optimoptions('linprog','Algorithm','dual-simplex','Display', 'off');
% options = optimoptions('linprog','Algorithm','interior-point','Display', 'off');
% [x,time] = mylinprog([],[],[],B,-b,zeros(n,1),[],[],options);
[x,time] = MySnoptLP(B,-b);

% Check if initial basis provides solution
if all(x>=-1e-6) && ~isempty(x)
    z(bas)=x(1:length(bas));
    z=z(1:n);
    return
else
    z = [];
end