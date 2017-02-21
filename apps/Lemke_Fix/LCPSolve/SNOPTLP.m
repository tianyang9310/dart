function [x,xlow,xupp,xmul,xstate, ...
    Flow,Fupp,Fmul,Fstate, ...
    ObjAdd,ObjRow,A,iAfun,jAvar,iGfun,jGvar] = SNOPTLP(B,q)


n      = length(q);
neF    = n;
Obj    = 0;
ObjRow = 0;

G = []; iGfun = []; jGvar = [];

A = B(:);

iAfun = repmat([1:n]',n,1);
jAvar = repmat([1:n],n,1);
jAvar = jAvar(:);

ObjAdd = 0;

% Initial x.

x      = ones (n,1);

xlow   = zeros(n,1);
xupp   = Inf*ones(n,1);

Flow   =  q;
Fupp   =  q;

xstate =     zeros(n,1);
xmul   =     zeros(n,1);

Fstate =   zeros(neF,1);
Fmul   =   zeros(neF,1);
end