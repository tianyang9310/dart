MaxIter = 1e4;
errList = [];
start = tic;
elapsedTime = 0;
count = 0;
for iter = 1:MaxIter
    idxContact = randi(4);
    idxContact = 4;
    poolSize = size(ct{idxContact,2},1);
    matSize = size(ct{idxContact,2},2);
    idxChoice = randi(poolSize);
    A = ct{idxContact,1}((idxChoice-1)*matSize+1:(idxChoice)*matSize,:);
    b = ct{idxContact,2}(idxChoice,:)';
    value = ct{idxContact,3}(idxChoice,:);
    [z,time]=LCPLPSOLVER(A,b,value);
    if ~isempty(z)
        count = count+1;
    end
    elapsedTime = elapsedTime + time;
end
% averagetime = toc(start)/MaxIter
averagetimeLP = elapsedTime/MaxIter
solvedRatio = count/MaxIter