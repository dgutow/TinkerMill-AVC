function [obj] = timeUpdate(obj,command)
%timeUpdate Summary of this function goes here
%   Detailed explanation goes here

% command a 2 element vector:
% 1. desired speed (m/s)
% 2. desired change in angle (radians)

% universal constants
METERS_PER_FOOT=0.3048;

%position
%orientation
%speed
%timeStep
%intertialSensorValue

% so let's try the simple approach, treat this as a massless particle
curVel = rot(-command(2))*[obj.speed;0];
newVel = command(1)*[1;0];
obj.intertialSensorValue=[command(2)/obj.timeStep+.01*(rand(1)-.5),(newVel-curVel)'/obj.timeStep+.3*(rand(1,2)-.5)];
%sobj.intertialSensorValue=[command(2)/obj.timeStep,(newVel-curVel)'/obj.timeStep];
obj.speed=command(1);
obj.orientation=obj.orientation+command(2);
obj.position=obj.position+obj.timeStep*obj.speed*[cos(obj.orientation),sin(obj.orientation)];
% [gyro (radians/s), forward acc, left acc (m/s^2)] 
end

function x = rot(angle) 
    x=[ cos(angle),-sin(angle);
        sin(angle), cos(angle)];
end