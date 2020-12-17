%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%                          %%%%%%%%%%
%%%%%%       Joe McCooey        %%%%%%%%%%
%%%%%%        P2522719          %%%%%%%%%%
%%%%%%      January 2020        %%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 
% This function should be run following the allocation of a server side port using V-REP
% Once connected, sensors are polled, before a logic state machine chooses
% one of the following states:
%   1. Map the inside of the room.
%   2. Once finished, move to room centre and then exit the room.
%   3. Once exited, map the environment before a beacon is found
%     (environment variable for task_3 is set within this function).
%   4. Or, map the room in its entirely.
%   5. Once either 3 or 4 is complete, move back to the centre of the room
% The above steps should all be performed whilst detecting, avoiding and
% mapping obstructions.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
function main()
% Find the VREP library
vrep=remApi('remoteApi');
% Close all open connections just in case
vrep.simxFinish(-1);
% Create a new simualation and connect
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

%Constants
MAX_X=16;
MAX_Y=16;
STANDARD_MOTOR_SPEED=0.9;

%% ENVIRONMENT VARIABLE FOR PERFORMING TASK 3 (Find Beason)
% Set to false to ignore beacon and map entire enviornment.
task3=true; 

%% Variables
mappedPoints=4*(ones(MAX_X,MAX_Y)); %size of the floor is 16x16, divide into 1m2 individual sections
%which have a value of
%-1: obstruction
% 0: target
% 1: robot
% 2: free space
% 3: unscanned
% 4: not yet needed
mappedPoints(7:11,6:11)=3; %inside of the room to be mapped first
obsUpdateNextLoop = zeros(1,2);
iteration = 0;
previousTarg=[0,0];
beaconPosition =[];
mapRoom=true;
mapCentre=[0,0];
scanFinished=false;
beaconFound=false;
atRoomCentre=false;

 %% SETUP GRID:      
p1=plot(0,0);
p2=plot(0,0);
% If a connection exists
if (clientID>-1)
    disp('Connected')
%% Set Object Handles
    [returnCode,robot]= vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_blocking);
    [returnCode,floor_visible_handle]=vrep.simxGetObjectHandle(clientID,'ResizableFloor_5_25',vrep.simx_opmode_blocking);
    % Create the objects handles to call the motors and sensors
    [returnCode,left_Motor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking);
    [returnCode,right_Motor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking);
    % Create the objects handles to call the front sensor array    
    %All sensors set to Ray type in V-REP - write in Report!    
    [returnCode,sensor1]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor1',vrep.simx_opmode_blocking);
    [returnCode,sensor2]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor2',vrep.simx_opmode_blocking);
    [returnCode,sensor3]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor3',vrep.simx_opmode_blocking);
    [returnCode,sensor4]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor4',vrep.simx_opmode_blocking);
    [returnCode,sensor5]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor5',vrep.simx_opmode_blocking);
    [returnCode,sensor6]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor6',vrep.simx_opmode_blocking);
    [returnCode,sensor7]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor7',vrep.simx_opmode_blocking);
    [returnCode,sensor8]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor8',vrep.simx_opmode_blocking);
    % Create the objects handles for the beacon
    [res, distHandle] = vrep.simxGetDistanceHandle(clientID, 'beacon', vrep.simx_opmode_blocking);
    [res, distance] = vrep.simxReadDistance(clientID, distHandle, vrep.simx_opmode_streaming);
    [res, beaconHandle] = vrep.simxGetObjectHandle(clientID, 'beacon',vrep.simx_opmode_blocking);
    
%% set up streams for robot position and orientation
    [returnCode, eulerAngles]=vrep.simxGetObjectOrientation(clientID, robot, floor_visible_handle, vrep.simx_opmode_streaming); %get absolute orientation in Euler angles
    [returnCode, glPosition]=vrep.simxGetObjectPosition(clientID, robot, floor_visible_handle, vrep.simx_opmode_streaming); %get absolute position
%% set up  all streams of the proximity sensors
    [returnCode,detectionStateSensor1,detectedPointSensor1,~,~]=vrep.simxReadProximitySensor(clientID,sensor1,vrep.simx_opmode_streaming);
    [returnCode,detectionStateSensor2,detectedPointSensor2,~,~]=vrep.simxReadProximitySensor(clientID,sensor2,vrep.simx_opmode_streaming);
    [returnCode,detectionStateSensor3,detectedPointSensor3,~,~]=vrep.simxReadProximitySensor(clientID,sensor3,vrep.simx_opmode_streaming);
    [returnCode,detectionStateSensor4,detectedPointSensor4,~,~]=vrep.simxReadProximitySensor(clientID,sensor4,vrep.simx_opmode_streaming);
    [returnCode,detectionStateSensor5,detectedPointSensor5,~,~]=vrep.simxReadProximitySensor(clientID,sensor5,vrep.simx_opmode_streaming);
    [returnCode,detectionStateSensor6,detectedPointSensor6,~,~]=vrep.simxReadProximitySensor(clientID,sensor6,vrep.simx_opmode_streaming);
    [returnCode,detectionStateSensor7,detectedPointSensor7,~,~]=vrep.simxReadProximitySensor(clientID,sensor7,vrep.simx_opmode_streaming);
    [returnCode,detectionStateSensor8,detectedPointSensor8,~,~]=vrep.simxReadProximitySensor(clientID,sensor8,vrep.simx_opmode_streaming);
%% main loop    
    while (true)
        %Turn on map and grid
        axis([1 MAX_X 1 MAX_Y])
        grid on;
        hold on;
        % Read the sonar sensors
        pollSensors;        
        %get all of the sensor inputs and put them into a matrix
        pointMatrix = [double(detectionStateSensor1)*norm(detectedPointSensor1)
            double(detectionStateSensor2)*norm(detectedPointSensor2)
            double(detectionStateSensor3)*norm(detectedPointSensor3)
            double(detectionStateSensor4)*norm(detectedPointSensor4)
            double(detectionStateSensor5)*norm(detectedPointSensor5)
            double(detectionStateSensor6)*norm(detectedPointSensor6)
            double(detectionStateSensor7)*norm(detectedPointSensor7)
            double(detectionStateSensor8)*norm(detectedPointSensor8)];
        
        %to allow sensors to initialise (*for some reason, v-rep sensors
        %don't read for the first few iterations)
        if iteration>100
            myPos = getGlobalPosition();
            %set new robot position as 1
            gp = round(myPos);
            %check the current location isn't an obstruction, if not give
            %matrix location a value of 1
            if mappedPoints(gp(1),gp(2))~=-1
                mappedPoints(gp(1),gp(2))=1;
            end
            moveToBeacon=false;

            
%% STATE LOGIC:
            % if robot is in the centre of the room, align with exit.
            if atRoomCentre 
                alignWithExit
                atRoomCentre=false;
            end  
            % if mapRoom is set to false (i.e. its already mapped) then...
            if ~mapRoom
                %... if task3 true, move to beacon if its close
                if task3
                    if getBeaconDistance(clientID)<1 && getBeaconDistance(clientID)>0  
                        glBeaconPosition = floor(getGLBeaconPosition());
                        target = [glBeaconPosition(1), glBeaconPosition(2)];
                        disp("beaconPosition = " + target);
                        moveToBeacon=true;
                    end
                end    
                %... or just continue to map environment
                if ~moveToBeacon&&ismember(3,mappedPoints)
                    [target,scanFinished] = findNearestUnscannedNode(mappedPoints,floor(myPos));
                    disp("previousTarg = "+previousTarg);
                    disp("target = "+target);
                    % if target is repeatedly set (i.e. keep hitting an
                    % object and can't move out of the way), reallocate target
                    if (target(1)==previousTarg(1))&&(target(2)==previousTarg(2))                        
                       mappedPoints(target(1),target(2))=2;
                       [target,scanFinished] = findNearestUnscannedNode(mappedPoints,floor(myPos));
                    end   
                end
                %go back to the centre node
                 if scanFinished||beaconFound
                     target=mapCentre; %%note: THIS IS NOT THE CENTRE, need to move an extra [0.5, 0.5]!!!!!
                 end
            end
            % map inside of room as it has not completely mapped yet
            if mapRoom   
                [target,roomMapFinished] = findNearestUnscannedNode(mappedPoints,floor(myPos));          
                % if target is repeatedly set (i.e. keep hitting an
                % object and can't move out of the way), reallocate target
                if (target(1)==previousTarg(1))&&(target(2)==previousTarg(2))
                    mappedPoints(target(1),target(2))=2;
                    [target,roomMapFinished] = findNearestUnscannedNode(mappedPoints,floor(myPos));
                end     
                % if inside of room is mapped, find the map centre and move
                % there
                if roomMapFinished
                    %set old robot position as free space 2
                    mappedPoints(mappedPoints==1)=2;
                    mappedPoints(mappedPoints==0)=2;
                    roomMapFinished=false;
                    mapCentre = floor(findCentre(mappedPoints));
                    target=mapCentre;                        
                    mappedPoints(mappedPoints==4)=3;
                    mapRoom=false;  
                    atRoomCentre=true;
                end 
            end
            
%% Check point matrix for obstructions
            [rotMat,bearing] = getOrientation();
            for i=1:length(pointMatrix)
                if pointMatrix(i) > 0
                    theta = getSensorAngle(i);
                    obstruction = getObstructionCoords(theta, pointMatrix(i),rotMat,myPos);
                    tmp=[1,1];
                    %set nodes as new  obstructions
                    mappedPoints(obstruction(1),obstruction(2))=-1;
                    %plot obstruction (17 - y coords to flip to correct way)
                    plot(obstruction(1),17-obstruction(2),'ro');
                    %find a new target if the previously allocated one is an obstruction,
                    %ad as long as target is not the beacon
                    
                    if target==obstruction
                        beacDist=getBeaconDistance(clientID);
                        if beacDist>1||beacDist==-1
                            target = findNearestUnscannedNode(mappedPoints,floor(myPos));
                            if target==floor(myPos)
                                target = findNearestUnscannedNode(mappedPoints,floor(getGlobalPosition()));
                            end
                        end
                    end
                    % obstruction needs to be updated on next loop if
                    % there's one in our current position                  
                    if obstruction==floor(getGlobalPosition)
                        obsUpdateNextLoop=obstruction;
                    end
                end
            end
%% Complete mapping
            % set target to zero, if there is one allocated
            if ~isempty(target)&&~isnan(target(1))
                mappedPoints(target(1),target(2))=0;
            else
                target=previousTarget;
                mappedPoints(target(1),target(2))=0;
            end    
            
            previousTarg=target; 
            
            %plot target and self (17 - ycoords to flip to correct way)
            delete(p1);
            delete(p2)           
            
            p1=plot(round(myPos(1)),round(17-myPos(2)),'bo');
            p2=plot(target(1),17-target(2),'go');
            hold on;
            
            %flip mapped points to make sense in matrix form 
            %(i.e. x=columns,y=rows)
            finalMapping = mappedPoints.';
            disp(finalMapping);
            % plan path
            [path,pathAvailable] = aStar(finalMapping,round(myPos'));            


            %follow path
            if pathAvailable               
                if ~isempty(path)
                    myPosNew=floor(myPos);
                    newCoords=path(1,:);
                    if myPosNew(1)==newCoords(1)&&myPosNew(2)==newCoords(2)
                        path(1,:)=[];
                    end
                end
                for i=1:size(path,1)
                    freeSpace= all(mappedPoints(path(i,:))==3);
                    targ= all(mappedPoints(path(i,:))==0);
                    me= all(mappedPoints(path(i,:))==1);
                    [moveComplete, obsPos] = moveToNewPos(path(i,:));
                    if moveComplete==true
                        %set old target as a free space, unless beacon, in which case, set to -1
                        if floor(beaconPosition) == mappedPoints(mappedPoints==1)
                            mappedPoints(mappedPoints==0)=-1;
                        elseif freeSpace==1||targ==1||me==1
                            mappedPoints(path(i,:))=2;
                        end
                    end
                    %if the move did not complete, there was an obsticle,
                    %break from the planned path and re-evaulate detection
                    %point matrix and target...
                    if moveComplete==false
                        %set old target as a free space, unless beacon, in which case, set to -1
                        if floor(beaconPosition) == mappedPoints(mappedPoints==0)
                            mappedPoints(mappedPoints==0)=-1;
                        else
                            mappedPoints(mappedPoints==0)=3;
                        end
                        if ~isnan(obsPos)
                            %mappedPoints(obsPos(1),obsPos(2))=-1;
                            pollSensors;                           
                            %get all of the sensor inputs and put them into a matrix
                            pointMatrix = [double(detectionStateSensor1)*norm(detectedPointSensor1)
                                double(detectionStateSensor2)*norm(detectedPointSensor2)
                                double(detectionStateSensor3)*norm(detectedPointSensor3)
                                double(detectionStateSensor4)*norm(detectedPointSensor4)
                                double(detectionStateSensor5)*norm(detectedPointSensor5)
                                double(detectionStateSensor6)*norm(detectedPointSensor6)
                                double(detectionStateSensor7)*norm(detectedPointSensor7)
                                double(detectionStateSensor8)*norm(detectedPointSensor8)];
                            
                            myPos = getGlobalPosition();
                            [rotMat,bearing] = getOrientation();
                            for k=1:length(pointMatrix)
                                if pointMatrix(k) > 0
                                    theta = getSensorAngle(k);
                                    obstruction = getObstructionCoords(theta, pointMatrix(k),rotMat,myPos);
                                    mappedPoints(obstruction(1),obstruction(2))=-1;
                                    %plot obstruction (17 - 7coords to flip to correct way)
                                    plot(obstruction(1),17-obstruction(2),'ro');
                                    %set nodes as new  obstructions                                    
                                    %mappedPoints(obsPos(1),obsPos(2))=-1;
                                end
                            end
                            %move away from obstruction to allow robot
                            %space to manouevre..                        
                            moveBackward;
                            pause(6.5);
                            break;
                        end
                    end
                end
            else
                moveToNewPos(target);
            end
            %set old robot position and target as free space 2
            mappedPoints(mappedPoints==1)=2;
            mappedPoints(mappedPoints==0)=2;
            %update obsttruction from last loop cycle
            if obsUpdateNextLoop(1)>0 || obsUpdateNextLoop(2)>0
                mappedPoints(obsUpdateNextLoop(1),obsUpdateNextLoop(2))=-1;
            end
            %if task 3 and move to beacon
            if moveToBeacon
                stop;     
                beaconFound=true;
            end    
            % Plot the data points
            %figure;
            %plot(mappedPoints,'o');
            %hold on;
        end
        iteration = iteration+1;
        
    end
    % Stop the simulation
    vrep.simxFinish(-1);
    
    disp('Failed connecting to remote API server');
end
% Call the destructor
vrep.delete();

%% NESTED FUNCTIONS

    function [orientationxy,bearing] = getOrientation()
        [returnCode, eulerAngles]=vrep.simxGetObjectOrientation(clientID, robot, floor_visible_handle, vrep.simx_opmode_buffer); %get absolute orientation in Euler angles
        bearing = rad2deg(eulerAngles(3));
        orientationxy = eul2rotm(eulerAngles); %convert to rotation matrix
    end

    function glPosition = getGlobalPosition()
        [returnCode, vrep_coords]=vrep.simxGetObjectPosition(clientID, robot, floor_visible_handle, vrep.simx_opmode_buffer); %get absolute position
        glPosition = [vrep_coords(1)+ 8.5,abs(vrep_coords(2) - 8.5)]; % convert from vrep coords(-7.5 to +7.5) to node coord system (1 to 16)
    end

    function glBeaconPosition = getGLBeaconPosition()
        glBeaconPosition = [beaconPosition(1)+ 8.5,abs(beaconPosition(2) - 8.5)]; % convert from vrep coords(-7.5 to +7.5) to node coord system (1 to 16)
    end

    function d = getBeaconDistance(clientID)
        threshold = 1;
        [res, distance] = vrep.simxReadDistance(clientID, distHandle, vrep.simx_opmode_buffer);
        [res, beaconPosition] = vrep.simxGetObjectPosition(clientID,beaconHandle, -1 , vrep.simx_opmode_blocking);
        if distance >= threshold
            d = -1;
        else
            d = distance;
        end
    end
    
    function alignWithExit
        [rotMat,bearing]=getOrientation();
        newAngle=180;        
        turnAngle = bearing - newAngle;
        % wrap
%         if turnAngle>180
%             turnAngle=turnAngle-180;
%         end  
        
        if bearing<0
            bearing=bearing+360;
        end
        if bearing<newAngle
            bearing=bearing+360;
        end    
        turnAngle=newAngle-bearing;
        if turnAngle>180
            turnAngle=turnAngle-360;
        end          
                 
        while (turnAngle<-1)||(turnAngle>1)
            [rotMat,bearing]=getOrientation();
            turnAngle = bearing - newAngle;
            if turnAngle>180
                turnAngle=turnAngle-180;
            end 
            if turnAngle < 0 && turnAngle > -180
                turnLeft;
                %         disp("turn left");
            else
                turnRight;
                %         disp("turn right");
            end  
        end
        moveForward;
        pause(12);
        stop;
    end    
    function [moveComplete, obsPos] = moveToNewPos(newPos)
        %newPos=[floor(newPos(1))+0.5,floor(newPos(2))+0.5];
        moveComplete=true;
        obsPos=[NaN,NaN];
        glPos=getGlobalPosition();
        [rotMat,bearing]=getOrientation();
        turnAngle=getTurnAngle(newPos(1),newPos(2),glPos(1),glPos(2),bearing);
        %turn to desired heading
        while (turnAngle<-1)||(turnAngle>1)
            [rotMat,bearing]=getOrientation();
            turnAngle=getTurnAngle(newPos(1),newPos(2),glPos(1),glPos(2),bearing);
            if turnAngle < 0 && turnAngle > -180
                turnRight;
            else
                turnLeft;
            end  
            pollSensors;           
            % if the sensors detect something ahead whilst turning, set moveComplete to false and stop. 
            if ((detectionStateSensor1==1)&&(norm(detectedPointSensor1)<0.2))||(detectionStateSensor2==1)&&(norm(detectedPointSensor2)<0.2)||(detectionStateSensor3==1)&&(norm(detectedPointSensor3)<0.2)||(detectionStateSensor4==1)&&(norm(detectedPointSensor4)<0.2)||(detectionStateSensor5==1)&&(norm(detectedPointSensor5)<0.2)||(detectionStateSensor6==1)&&(norm(detectedPointSensor6)<0.2)||(detectionStateSensor7==1)&&(norm(detectedPointSensor7)<0.2)||(detectionStateSensor8==1)&&(norm(detectedPointSensor8)<0.2)                  
                moveComplete=false;
                disp("OBJECT DETECTED AHEAD!!!");
                obsPos=[newPos(1),newPos(2)];
                disp("obsPos = " + obsPos);
                stop;
                break;
            else
                obsPos=[NaN NaN];
            end
            
        end
        stop;
        
        %if turn was successful move towards the disired point
        if moveComplete==true
            glPos=getGlobalPosition();
            distance=getDistance(newPos(1),newPos(2),glPos(1),glPos(2));      
            while distance>0.5 && distance<2.8
                glPos=getGlobalPosition();
                distance=getDistance(newPos(1),newPos(2),glPos(1),glPos(2));
                pollSensors;                
                if ((detectionStateSensor1==1)&&(norm(detectedPointSensor1)<0.2))||(detectionStateSensor2==1)&&(norm(detectedPointSensor2)<0.2)||(detectionStateSensor3==1)&&(norm(detectedPointSensor3)<0.2)||(detectionStateSensor4==1)&&(norm(detectedPointSensor4)<0.2)||(detectionStateSensor5==1)&&(norm(detectedPointSensor5)<0.2)||(detectionStateSensor6==1)&&(norm(detectedPointSensor6)<0.2)||(detectionStateSensor7==1)&&(norm(detectedPointSensor7)<0.2)||(detectionStateSensor8==1)&&(norm(detectedPointSensor8)<0.2)               
                    moveComplete=false;
                    disp("OBJECT DETECTED AHEAD!!!");
                    obsPos=[newPos(1),newPos(2)];
                    disp("obsPos = " + obsPos);
                    stop;
                    break;
                else
                    obsPos=[NaN NaN];
                end
                % if desired point is reached within 0.5m, stop and set
                % moveComplete to true
                if (glPos(1)<newPos(1)+0.5)&&(glPos(1)>newPos(1)-0.5)&&(glPos(2)<newPos(2)+0.5)&&(glPos(2)>newPos(2)-0.5)
                    stop;
                    moveComplete=true;
                    disp("MOVE COMPLETE!");
                    break;
                else 
                    moveForward;
                end
            end
        end
    end

    function pollSensors
        [returnCode,detectionStateSensor1,detectedPointSensor1,~,~]=vrep.simxReadProximitySensor(clientID,sensor1,vrep.simx_opmode_buffer);
        [returnCode,detectionStateSensor2,detectedPointSensor2,~,~]=vrep.simxReadProximitySensor(clientID,sensor2,vrep.simx_opmode_buffer);
        [returnCode,detectionStateSensor3,detectedPointSensor3,~,~]=vrep.simxReadProximitySensor(clientID,sensor3,vrep.simx_opmode_buffer);
        [returnCode,detectionStateSensor4,detectedPointSensor4,~,~]=vrep.simxReadProximitySensor(clientID,sensor4,vrep.simx_opmode_buffer);
        [returnCode,detectionStateSensor5,detectedPointSensor5,~,~]=vrep.simxReadProximitySensor(clientID,sensor5,vrep.simx_opmode_buffer);
        [returnCode,detectionStateSensor6,detectedPointSensor6,~,~]=vrep.simxReadProximitySensor(clientID,sensor6,vrep.simx_opmode_buffer);
        [returnCode,detectionStateSensor7,detectedPointSensor7,~,~]=vrep.simxReadProximitySensor(clientID,sensor7,vrep.simx_opmode_buffer);
        [returnCode,detectionStateSensor8,detectedPointSensor8,~,~]=vrep.simxReadProximitySensor(clientID,sensor8,vrep.simx_opmode_buffer);
    end

    function moveForward
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,STANDARD_MOTOR_SPEED,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,STANDARD_MOTOR_SPEED,vrep.simx_opmode_blocking);
    end

    function moveBackward
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,-STANDARD_MOTOR_SPEED,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,-STANDARD_MOTOR_SPEED,vrep.simx_opmode_blocking);
    end

    function turnLeft
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,0,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,STANDARD_MOTOR_SPEED,vrep.simx_opmode_blocking);
    end

    function turnRight
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,0,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,STANDARD_MOTOR_SPEED,vrep.simx_opmode_blocking);
    end

    function stop
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,0,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,0,vrep.simx_opmode_blocking);
    end

end

