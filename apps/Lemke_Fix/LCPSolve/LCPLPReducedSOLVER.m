function [z,time]=LCPLPReducedSOLVER(A,b,value)

numContactsToLearn = length(value);
numBasis = 8;
% options = optimoptions('linprog','Algorithm','dual-simplex','Display', 'off');
options = optimoptions('linprog','Algorithm','interior-point','Display', 'off');
ub_index = [];
for i = 1:numContactsToLearn
    if (value(i) == 9)
        ub_index = [ub_index (numBasis+1)*numContactsToLearn+i];
    elseif (value(i) == 8)
        ub_index = [ub_index i numContactsToLearn+((i-1)*numBasis+1:i*numBasis)];
    else
        offset = numContactsToLearn+(i-1)*numBasis;
        ub_index = [ub_index i offset+value(i)+1 (numBasis+1)*numContactsToLearn+i];
    end
end

LP_lb = zeros(length(ub_index),1);
% LP_ub = zeros(numContactsToLearn*(numBasis+2),1);

T = zeros(1,numContactsToLearn*(numBasis+2));
TT = zeros(numContactsToLearn*(numBasis+2),length(ub_index));
for i = 1:length(ub_index)
    TT(ub_index(i),i) = 1;
end
T(ub_index) = 1;


LP_Aequ = T*A*TT;
LP_bequ = -T*b;
LP_Ainequ = - A*TT;
LP_binequ = b;

% subStart = tic;
[z_sub,time] = mylinprog([],LP_Ainequ,LP_binequ,LP_Aequ,LP_bequ,LP_lb,[],[],options);
% elapsedTime = toc(subStart);

z = zeros(numContactsToLearn*(numBasis+2),1);
if ~isempty(z_sub)
    z(ub_index) = z_sub;
else 
    z =[];
end

end