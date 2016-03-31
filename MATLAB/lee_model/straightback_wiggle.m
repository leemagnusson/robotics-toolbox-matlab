function [ q, t ] = straightback_wiggle( dt )
%straightback_wiggle Generates a sinusoidal wiggling motion with the robot
%in a straight out pose

qstraightback = [-3*pi/8,0,-pi/2,0,0,0,-80*pi/180,0,0];

t = (0:dt:10)';
q = ones(size(t)) * qstraightback  + sin(t) * ones(size(qstraightback));


end

