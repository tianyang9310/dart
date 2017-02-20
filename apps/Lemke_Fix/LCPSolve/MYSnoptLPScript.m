snscreen on;
snprint('MySNOPTLP.out');  % By default, screen output is off;

MYSNOPTLP.spc = which('MYSNOPTLP.spc');
snspec (MYSNOPTLP.spc);

snseti ('Major Iteration limit', 250);

[x,xlow,xupp,xmul,xstate, ...
    Flow,Fupp,Fmul,Fstate, ...
    ObjAdd,ObjRow,A,iAfun,jAvar,iGfun,jGvar] = MySNOPTLP(B,-b);

[x,F,INFO,xmul,Fmul,xstate,Fstate,output]= snopt( x, xlow, xupp, xmul, xstate, ...
    Flow, Fupp, Fmul, Fstate, ...
    @t1dietuserfun, ObjAdd, ObjRow, ...
    A, iAfun, jAvar, iGfun, jGvar ...
    );

snprint off;
snend;
