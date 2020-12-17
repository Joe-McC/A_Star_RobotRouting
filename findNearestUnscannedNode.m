%%%%%%%%%%%%%%%%%% findNearestUnscannedNode %%%%%%%%%%%%%%%%%%%%%%%%%%%
% this function does uses an expanding tree method to check a dist of one
% node in each direction to search for an unscanned node. If none are
% found, these nodes are moved into the top of the checked list. The ever 
% expanding check list is loop through until an unscanned node is found
% 
function [target,finishedScan] = findNearestUnscannedNode(mappedPoints,glPosition)
%max bounds of the map
MAX=16;
target = [NaN NaN];
finishedScan = false;
j=1;
CHECKED =ones(20000,2); %..
CHECKED(1,1) = glPosition(1);
CHECKED(1,2) = glPosition(2);

for i=1:length(CHECKED)
    if CHECKED(i,1) + 1 <= MAX             
        if (mappedPoints(CHECKED(i,1)+1,CHECKED(i,2)) == 3)
            target = [CHECKED(i,1)+1, CHECKED(i,2)];
            break
        else  
            row = [CHECKED(i,1)+1,CHECKED(i,2)];
            [CHECKED,j]=addRow(row,CHECKED,j);    
        end    
    end    
    if CHECKED(i,1) - 1 > 0
        if (mappedPoints(CHECKED(i,1)-1,CHECKED(i,2)) == 3)
            target = [CHECKED(i,1)-1,CHECKED(i,2)];
            break
        else             
            row = [CHECKED(i,1)-1,CHECKED(i,2)];
            [CHECKED,j]=addRow(row,CHECKED,j); 
        end
    end
    if CHECKED(i,2) + i <= MAX
        if (mappedPoints(CHECKED(i,1),CHECKED(i,2)+1) == 3)
            target = [CHECKED(i,1), CHECKED(i,2)+1];
            break
        else  
            row = [CHECKED(i,1),CHECKED(i,2)+1];
            [CHECKED,j]=addRow(row,CHECKED,j);                        
        end
    end
    if CHECKED(i,2)- 1 > 0
        if (mappedPoints(CHECKED(i,1),CHECKED(i,2)-1) == 3)
            target = [CHECKED(i,1), CHECKED(i,2)-1];
            break
        else              
            row = [CHECKED(i,1),CHECKED(i,2)-1];
            [CHECKED,j]=addRow(row,CHECKED,j);
        end
    end  
    if(CHECKED(i,1) + 1 <= MAX)&&(CHECKED(i,2) + 1 <= MAX)        
        if (mappedPoints(CHECKED(i,1)+1,CHECKED(i,2)+1) == 3)
            target = [CHECKED(i,1)+1,CHECKED(i,2)+1];
            break 
        else  
            row = [CHECKED(i,1)+1,CHECKED(i,2)+1];
            [CHECKED,j]=addRow(row,CHECKED,j);
        end    
    end    
    if(CHECKED(i,1) + 1 <= MAX)&&(CHECKED(i,2) -1 > 0)      
        if (mappedPoints(CHECKED(i,1)+1,CHECKED(i,2)-1) == 3)
            target = [CHECKED(i,1)+1, CHECKED(i,2)-1];            
            break
        else    
            row = [CHECKED(i,1)+1,CHECKED(i,2)-1];
            [CHECKED,j]=addRow(row,CHECKED,j);
        end    
    end    
    if(CHECKED(i,1) - 1 > 0)&&(CHECKED(i,2) + 1 <= MAX)        
        if (mappedPoints(CHECKED(i,1)-1,CHECKED(i,2)+1) == 3)
            target = [CHECKED(i,1)-1, CHECKED(i,2)+1];
            break
        else  
            row = [CHECKED(i,1)-1,CHECKED(i,2)+1];
            [CHECKED,j]=addRow(row,CHECKED,j); 
        end  
    end    
    if(CHECKED(i,1) - 1 > 0)&&(CHECKED(i,2) - 1 > 0)        
        if (mappedPoints(CHECKED(i,1)-1,CHECKED(i,2)-1) == 3)
            target = [CHECKED(i,1)-1, CHECKED(i,2)-1];
            break
        else  
            row = [CHECKED(i,1)-1,CHECKED(i,2)-1];
            [CHECKED,j]=addRow(row,CHECKED,j);
        end    
    end 
end    
if isnan(target(1))
    finishedScan = true;
end
end

