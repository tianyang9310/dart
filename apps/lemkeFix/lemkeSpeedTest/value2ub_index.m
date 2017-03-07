function ub_index = value2ub_index(value)
numContactsToLearn = length(value);
numBasis = 4;

ub_index = [];

for i = 1:numContactsToLearn
    if (value(i) == 9)
        ub_index = [ub_index (numBasis+1)*numContactsToLearn+i];
    elseif (value(i) == 8)
        ub_index = [ub_index i numContactsToLearn+((i-1)*numBasis+1:i*numBasis)];
    else
        offset = numContactsToLearn+(i-1)*numBasis;
%         findex = value(i) + 1;
%         findex = findex - 1 : findex + 1;
%         for adjIdx = 1:3
%             if (findex(adjIdx)<1)
%                 findex(adjIdx) = findex(adjIdx) + 8;
%             elseif (findex(adjIdx)>8)
%                 findex(adjIdx) = findex(adjIdx) - 8;
%             end
%         end
%         ub_index = [ub_index i offset+findex (numBasis+1)*numContactsToLearn+i];        
        ub_index = [ub_index i (numBasis+1)*numContactsToLearn+i];  
        ub_index = [ub_index offset+floor(value(i)/2)+1];
        if (mod(value(i),2) ~= 0) 
            ub_index = [ub_index offset+mod(ceil(value(i)/2),4)+1];
        end
    end
end
end