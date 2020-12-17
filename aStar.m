%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%% A* ALGORITHM %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function implments the A* Search Algorithm using the mapped points
% to create an optimal path based on a cost function f(n) which is the sum
% of the cost to a node g(n) plus the heuristic cost to the target h(n).
% In this case, the h(n) cost is calculated as the Euclidean distance from
% the node to the targety
%%
function [path,pathAvailable] = aStar(mappedPoints,myPos)
%Constants
MAX_X=16;
MAX_Y=16;
% axis([1 MAX_X+1 1 MAX_Y+1])
% grid on;
% hold on;

% Initialize variables and lists
openSet=[];
closedSet=[];
pathAvailable=false;

noTarget = true;
closedCount=1;%Dummy counter
for i=1:MAX_Y
    for j=1:MAX_X
        % Initialize the closed list with current obstacles
        if(mappedPoints(i,j) == -1)
            closedSet(closedCount,2)=i;
            closedSet(closedCount,1)=j;
            closedCount=closedCount+1;
        end
        if(mappedPoints(i,j) == 0)
            targetX = j;
            targetY = i;
            noTarget =false;
        end
    end
end

% self position
myX = myPos(1);
myY = myPos(2);

if noTarget==true
    targetX = myX;
    targetY = myY; 
end
%initialise variables
fScore = getDistance(myX,myY,targetX,targetY);
gScore = 0;
neighbour_fScore=1000; %large value
newRow = [myX myY gScore fScore];
% openCount=1;
% openSet(openCount,:)=newRow;
openSet = [];
openCount=0;

nodeCount=1;
nodes=[myX myY gScore fScore];
% While target is not found
while (true)
% find the node with the least f on       
    if isempty(openSet)==0        
        [~, rowInd] = min(openSet(:,4));
        nodeCount = nodeCount+1;
%       nodes recorded in the path so far
        tmp1 = openSet(rowInd,1:2);
        oldNodeRow=(ismember(tmp1,nodes(:,1:2),'rows'));
        oldNode = all(oldNodeRow);
        %if lowest f open set node is already a path node, remove
        while oldNode==1
            tmp1 = openSet(rowInd,1:2);
            oldNodeRow=(ismember(tmp1,nodes(:,1:2),'rows'));
            oldNode = all(oldNodeRow);
            openSet(rowInd,:)=[0 0 100 100];  
            [~, rowInd] = min(openSet(:,4));
        end    
        openSetPositions=openSet(:,1:2);
%% THE FOLLOWING CODE TO RECTIFY DEAD END PATHS DOES NOT CURRENTLY WORK        
        %if the path is a dead end, add the nodes in the list (except for
        %my position) to the closed set, and empty from node array
%THIS FEATURE NEEDS FURTHER TESTING, SEEMS TO GET INTO A LOOP AND THEN
%EXCEED BOUNDS
%         if ~all(openSetPositions)
%             for i=2:nodeCount
%                 closedSet(closedCount,:)=nodes(i,1:2);
%                 closedCount=closedCount+1;
%             end                         
%             nodes(2:end,:)=[];
%             nodeCount=1;
%         end    
%%
        %set the lowest f open set node to the next path node
        nodes(nodeCount,:) = openSet(rowInd,:);
        %set the current path node to closed set
        closedSet(closedCount,1:2)=nodes(nodeCount,1:2);
        closedCount=closedCount+1;
%% THE FOLLOWING CODE TO RECTIFY DEAD END PATHS DOES NOT CURRENTLY WORK   
        %THIS FEATURE NEEDS FURTHER TESTING, SEEMS TO MESS UP ROUTING FOR
        %MORE DIFFICULT ROUTES
%     else
%         if nodeCount>1
%             openSetPositions=openSet(:,1:2);
%             %if the path is a dead end, add the nodes in the list (except for
%             %my position) to the closed set, and empty from node array
%             if ~all(openSetPositions)
%                 for i=2:nodeCount
%                     closedSet(closedCount,:)=nodes(i,:);
%                     closedCount=closedCount+1;
%                 end                         
%                 nodes(2:end,:)=[];
%                 nodeCount=1;
%             end   
%         end
    end    
%    if the next node is the target, break out of loop and return
%    pathAvailble = true and the current set of recorded nodes
    if (nodes(nodeCount,1)==targetX)&&(nodes(nodeCount,2)==targetY)
        pathAvailable=true;
        break
    end    
%    remove from openSet
    openSet = [];
    openCount=0;
%    generate 8 successors in each direction
    for i= 1:-1:-1
        for j= 1:-1:-1
            if (i~=j || i~=0)  %The node itself is not its successor
                sucX = nodes(nodeCount,1)+i;
                sucY = nodes(nodeCount,2)+j;
                if( (sucX>0&&sucX<=MAX_X)&&(sucY>0&&sucY<=MAX_Y))%node within map bound   
                    suc = [sucX sucY];
                    if isempty(closedSet)==1
                        sucIsInClosedSet=0;
                    else    
                        temp=(ismember(suc,closedSet(:,1:2),'rows'));
                        sucIsInClosedSet = all(temp);
                    end    
%                     if sucIsInClosedSet==1
%                         disp("closedSetMatch");
%                     end
                    if sucIsInClosedSet==0     
                        %g score is the cost to the current node, plus the
                        %cost to the potential node
                        gScore=nodes(nodeCount,3)+getDistance(nodes(nodeCount,1),nodes(nodeCount,2),sucX,sucY);%cost of travelling to node                                
                        %h score is the heuristic cost function from
                        %potential node to target
                        hScore=getDistance(targetX,targetY,sucX,sucY);%distance between successor and goal
                        fScore=gScore+hScore;
%                       if the gScore of the suggested move is better
%                       than the others, record.                         
                        %neighbour_fScore=fScore;
                        openCount=openCount+1;
                        openSet(openCount,1)=sucX;
                        openSet(openCount,2)=sucY;
                        openSet(openCount,3)=gScore;%gn
                        openSet(openCount,4)=fScore;%fn
                        %end    
                    end
                end
            end
        end
    end
end   
path = nodes(2:end,1:2);
end