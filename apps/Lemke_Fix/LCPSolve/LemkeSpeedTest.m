MaxIter = 1e4;
errList = [];
start = tic
for iter = 1:MaxIter
    idxContact = randi(4);
    idxContact = 4;
    poolSize = size(ct{idxContact,2},1);
    matSize = size(ct{idxContact,2},2);
    idxChoice = randi(poolSize);
    A = ct{idxContact,1}((idxChoice-1)*matSize+1:(idxChoice)*matSize,:);
    b = ct{idxContact,2}(idxChoice,:)';
    [z,err]=LEMKE(A,b);
    errList = [errList err];
end
averagetimeLEMKE = toc(start)/MaxIter
solvedRatio = length(find(errList==0))/MaxIter