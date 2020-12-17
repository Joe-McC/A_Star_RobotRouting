%%%%%%%%%%%%%%%%%%%%%%%% getTurnAngle %%%%%%%%%%%%%%%%%%%%%%%
%This function calculates the angle between any two cartesian 
%coordinates.
function turnAngle = getTurnAngle(x1,y1,x2,y2,orientation)
theta_rad = -atan2(y1-y2,x1-x2);%flip the map to work in VREPs orientation!!!
theta = rad2deg(theta_rad);
%wrapping theta and orientation to 0 -> 360 degrees
if theta<0
    theta=theta+360;
end
if orientation<0
    orientation=orientation+360;
end
% ensuring orientation is always greater
if orientation<theta
    orientation=orientation+360;
end    
turnAngle=theta-orientation;
%convert back to -180 to 180 degrees spectrum
if turnAngle>180
    turnAngle=turnAngle-360;
end    

end