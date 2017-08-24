function [z,err,time] = LEMKE(M,q,z0)
% LEMKE Solves linear complementarity problems (LCPs).
% An LCP solves
% Mz+q >=0, z>=0, z'(Mz+q)=0.
% USAGE:
% [z,err]=LEMKE(M,q,z0);
% The input z0 defines a starting basis; it can be either
% an initial guess of the solution or a vector of zeros and ones
% with ones representing those z(i) thought to be non-zero in the
% solution. For example, passing z=[1.5;0;2.2] has the same
% effect as passing z=[1;0;1].
% If z0 is omitted the origin is used as a starting basis.
% ERR returns an error condition:
% 0: Solution found
% 1: Maximum iterations exceeded
% 2: Unbounded ray termination
% 3: Initial basis cannot be computed - try new value of z0
% If NARGOUT==1, a warning message is displayed instead.
%
% ALGORITHM
% Uses a modified Lemke's algorithm (complementary pivoting)
% with a covering ray of ones. The algorithm is modified to
% allow a user defined initial basis.

% Copyright (c) 1997 by Paul L. Fackler

%tic; flops(0);
Lemke_start=tic;
n = length(q);

if nargin<4, ineqind=ones(n,1); end
if nargin<3, z0=[]; end

if size(M)~=[n n]
  error('Matrices are not compatible');
end

zer_tol = 1e-6;
piv_tol = 1e-12;
maxiter = min(1000,25*n);
err=0;

% Trivial solution exists
if all(q>=0)
  z=zeros(n,1); return;
end

% Initializations
% (note: memory allocation for all variables to avoid later reallocation)
z = zeros(2*n,1);
j=zeros(n,1);
iter=0;
theta=0;
ratio=0;
leaving=0;
Be=ones(n,1);
x=q;
bas=zeros(n,1);
nonbas=zeros(n,1);

t = 2*n+1; % Artificial variable
entering=t; % is the first entering variable

% Determine initial basis
if isempty(z0)
  bas=[];
  nonbas=(1:n)';
else
  bas=find(z0>piv_tol);
  nonbas=find(z0<=piv_tol);
end

B=spalloc(n,n,nnz(M)); % allocate memory for B
B=-speye(n);

% Determine initial values
if ~isempty(bas)
  B=[M(:,bas) B(:,nonbas)];
  if condest(B)>1e16
    z=[]; err=3; 
    % return
%     B = B + 1e-3*eye(n);
  else 
  x=-(B\q);
  end
% x = lsqlin(B,-q,[],[],[],[],zeros(n,1),[]);
% options = optimoptions('linprog','Algorithm','interior-point','Display', 'off');
% [x,time] = mylinprog([],[],[],B,-q,zeros(n,1),[],[],options);
end

% Check if initial basis provides solution
if all(x>=0)
  z(bas)=x(1:length(bas));
  z=z(1:n);
  return
end

% Determine initial leaving variable
[tval,lvindex]=max(-x);
bas=[bas;(n+nonbas)];
leaving=bas(lvindex);

bas(lvindex)=t; % pivot in the artificial variable

U=x<piv_tol;
%U=ones(n,1); % Alternative artificial vector
Be=-(B*U);
x=x+tval*U;
x(lvindex)=tval;
B(:,lvindex)=Be;

% Main iterations begin here
for iter=1:maxiter
  % Check if done; if not, get new entering variable
  if (leaving == t) break
  elseif (leaving <= n)
    entering = n+leaving;
    Be=zeros(n,1); Be(leaving)=-1;
  else
    entering = leaving - n;
    Be = M(:,entering);
  end
  d = B\Be;

  % Find new leaving variable
  j=find(d>piv_tol); % indices of d>0
  if isempty(j) % no new pivots - ray termination
    err=2;
    break
  end
  theta=min((x(j)+zer_tol)./d(j)); % minimal ratios, d>0
  j=j(find((x(j)./d(j))<=theta)); % indices of minimal ratios, d>0
  lvindex=find(bas(j)==t); % check if artificial among these
  if ~isempty(lvindex) % Always use artifical if possible
    lvindex=j(lvindex);
  else % otherwise pick among set of max d
    [theta,lvindex]=max(d(j));
    lvindex=find(d(j)==theta);
    lvindex=ceil(length(lvindex)*rand(1,1)); % if multiple choose randomly
    if isempty(j)
        disp('error')
        err = 2;
        break;
    end
    lvindex=j(lvindex);
  end
  leaving=bas(lvindex);

  % Perform pivot
  ratio=x(lvindex)./d(lvindex);
  x = x - ratio*d;
  x(lvindex) = ratio;
  B(:,lvindex) = Be;
  bas(lvindex) = entering;
end % end of iterations
if iter>=maxiter & leaving~=t err=1; end

%if err==0
  z(bas) = x; z = z(1:n);
%else
% z=[];
%end

time = toc(Lemke_start);
%t=toc; f=flops;
% disp([num2str(iter) ' ' num2str(t) ' ' num2str(f)])


% Display warning messages if no error code is returned
if nargout<2 & err~=0
  s='Warning: solution not found - ';
  if err==1
    disp([s 'Iterations exceeded limit']);
  elseif err==2
    disp([s 'Unbounded ray']);
  elseif err==3
    disp([s 'Initial basis infeasible']);
  end
end