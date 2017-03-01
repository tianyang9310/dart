function ub_index = value2ub_index(value)
numContactsToLearn = length(value);
numBasis = 8;

% ub_index = [];

ub_index = {[]};


for i = 1:numContactsToLearn
    if (value(i) == 9)
        for each = 1:length(ub_index)
            ub_index{each} = [ub_index{each} (numBasis+1)*numContactsToLearn+i];
        end
    elseif (value(i) == 8)
        for each = 1:length(ub_index)
            ub_index{each} = [ub_index{each} i numContactsToLearn+((i-1)*numBasis+1:i*numBasis)];
        end
    else
        curNum = length(ub_index);
        for each = 1:curNum
            offset = numContactsToLearn+(i-1)*numBasis;
            old_ub_index = ub_index{each};
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
            ub_index{each} = [old_ub_index i offset+value(i)+1 (numBasis+1)*numContactsToLearn+i];
            new_item = value(i)-1;
            while (new_item < 0)
                new_item = new_item + 8;
            end
            new_item = new_item + 1;
            ub_index{end+1} =  [old_ub_index i offset+new_item offset+value(i)+1 (numBasis+1)*numContactsToLearn+i];
            new_item = value(i)+1;
            while (new_item > 7)
                new_item = new_item - 8;
            end
            new_item = new_item + 1;
            ub_index{end+1} =  [old_ub_index i offset+value(i)+1 offset+new_item (numBasis+1)*numContactsToLearn+i];
        end
    end
end
end