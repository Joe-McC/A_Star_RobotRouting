function [CHECKED,j] = addRow(row,CHECKED,j)
    rowPresent = ismember(row,CHECKED,'rows');
    if rowPresent == 0
        CHECKED(j,1)=row(1);
        CHECKED(j,2)=row(2);
        j=j+1;
    end   
end

