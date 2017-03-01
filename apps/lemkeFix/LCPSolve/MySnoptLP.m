function [z, time] = MySnoptLP(A,b)
snscreen off;
snprint('MySNOPTLP.out');  % By default, screen output is off;

MYSNOPTLP.spc = which('MYSNOPTLP.spc');
snspec (MYSNOPTLP.spc);

snseti ('Major Iteration limit', 250);

[x,xlow,xupp,xmul,xstate, ...
    Flow,Fupp,Fmul,Fstate, ...
    ObjAdd,ObjRow,A,iAfun,jAvar,iGfun,jGvar] = SNOPTLP(A,b);
MySnopt = tic;
[z,F,INFO,xmul,Fmul,xstate,Fstate,output]= snopt( x, xlow, xupp, xmul, xstate, ...
    Flow, Fupp, Fmul, Fstate, ...
    @placeholder, ObjAdd, ObjRow, ...
    A, iAfun, jAvar, iGfun, jGvar ...
    );
time = toc(MySnopt);
snprint off;
snend;



end
