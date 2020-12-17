function theta = getSensorAngle(sensorNo)
angles = [-90, -50, -30, -10, 10, 30, 50, 90];
theta = deg2rad(angles(sensorNo));
end


