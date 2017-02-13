sortdata = sort(lcpdata2_trim,1);
MaxData = sortdata(end-5,:);
MinData = sortdata(5,:);
MaxA = MaxData(1:171);
MinA = MinData(1:171);
MaxAFull = zeros(18,18);
MinAFull = zeros(18,18);
for i = 1 : 18
    for j = i : 18
        MaxAFull(i,j) = MaxA((38-i)*(i-1)/2+j-i+1);
        MinAFull(i,j) = MinA((38-i)*(i-1)/2+j-i+1);
    end
end
MaxAFull = MaxAFull + triu(MaxAFull,1)';
MinAFull = MinAFull + triu(MinAFull,1)';
Maxb = MaxData(174:end-2);
Minb = MinData(174:end-2);
csvwrite('MAXMIN_A.csv',[MaxAFull;zeros(1,18);MinAFull]);
csvwrite('MAXMIN_b.csv',[Maxb;zeros(1,18);Minb]);