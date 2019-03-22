function [ r ] = RotM2RPY( R )
%RotM2RPY Findes RPY Angles in deg
%   From http://planning.cs.uiuc.edu/node103.html & http://planning.cs.uiuc.edu/node102.html

rz = atan2(R(2,1),R(1,1))*180/pi;

ry = atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2))*180/pi;

rx = atan2(R(3,2),R(3,3))*180/pi;

r=[rx, ry, rz];

end

