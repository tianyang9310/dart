MaxIter = 1e4;
errList = [];
start = tic;
elpasedTime = 0;
for iter = 1:MaxIter
    idxContact = randi(12);
%     idxContact = 4;
    poolSize = size(ct{idxContact,2},1);
    matSize = size(ct{idxContact,2},2);
    idxChoice = randi(poolSize);
    A = ct{idxContact,1}((idxChoice-1)*matSize+1:(idxChoice)*matSize,:);
    b = ct{idxContact,2}(idxChoice,:)';
%     value = ct{idxContact,3}(idxChoice,:);
%     ub_index = value2ub_index(value);
%     z0 = zeros(size(b));
%     z0(ub_index) = 1;
%     [z,err]=LEMKE(A,b,z0);
    [z,err,time]=LEMKE(A,b);
%     if any(abs(z)<1e-6) && any(abs(z)>0)
%         z
%     end
    errList = [errList err];
    elpasedTime = elpasedTime + time;
end
averagetimeLEMKE = toc(start)/MaxIter
averagetimeLEMKE = elpasedTime/MaxIter
solvedRatio = length(find(errList==0))/MaxIter