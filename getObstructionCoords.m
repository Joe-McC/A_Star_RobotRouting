%%%%%%%%%%%%%%%%%% getObstructionCoords %%%%%%%%%%%%%%%%%%%%%%%%%%%
% getObstructionCoords takes the local distance and angle of a 
% detected object and translates it to the global coordinate system

function obstruction = getObstructionCoords(theta, r, orientation, glPosition)
ROBOT_RADIUS=0.5;
%find the distance of the object relative to robot
xs = cos(theta) * (r + ROBOT_RADIUS);
ys = sin(theta) * (r + ROBOT_RADIUS);

%rotate to global coordinate
%gcs_rotate = ([xs, ys, 0] * orientation);
gcs_rotate = ([xs, ys] * orientation(2:3,2:3));

%translate to global coordinate & fit to node
gcs_trans= [floor(glPosition(1)) + round(gcs_rotate(1)), floor(glPosition(2)) + round(gcs_rotate(2))];
% check if in bounds
if gcs_trans(1) < 1
    gcs_trans(1)=gcs_trans(1)+1;
end
if gcs_trans(2) < 1
    gcs_trans(2)=gcs_trans(2)+1;   
end
if gcs_trans(1) > 16
    gcs_trans(1)=gcs_trans(1)-1;
end
if gcs_trans(2) > 16
    gcs_trans(2)=gcs_trans(2)-1;   
end
obstruction = [gcs_trans(1),gcs_trans(2)];

end
