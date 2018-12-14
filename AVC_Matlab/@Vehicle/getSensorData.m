function [obj] = getSensorData(obj)
%GETSENSORS Summary of this function goes here
%   Detailed explanation goes here

% universal constants
METERS_PER_FOOT=0.3048;
% sensor parameters
LIDAR_RANGE=12;
LIDAR_NUM_SAMPLES=180;

persistent cl;
if isempty(cl)
    load course.mat;
    %cl - course lines
    cl=wallLines;
end

%position
%orientation
%speed
%timeStep
%intertialSensorValue

% sensors are
% 1. LIDAR
% 2. DELTA POSITION AND ORIENTATION (BODY FRAME)

% LIDAR
angles=linspace(0,2*pi,LIDAR_NUM_SAMPLES+1)+obj.orientation;
angles(end)=[];

%ll - lidar lines
ll=ones(LIDAR_NUM_SAMPLES,1)*obj.position;
ll(:,[3,4])=[cos(angles)',sin(angles)'];

% now for all of the lines find their intersecton with the courseLines.
clInds=find(sum((cl(:,[1,2])-obj.position).^2,2).^.5<1.5*LIDAR_RANGE);


% lines are going to be represented as c_i1+c_i2*si for all s in [0,1]
% intersection of two lines [c(1);c(2)]+[c(3);c(4)]*s1=[d(1);d(2)]+[d(3);d(4)]*s2 ->
% [c(1)-d(1)]=[-c(3),d(3)]*[s1]
% [c(2)-d(2)]=[-c(4),d(4)]*[s2] ->
% s1 = -(c1*d4 - c2*d3 - d1*d4 + d2*d3)/(c3*d4 - c4*d3)
% s2 = -(c1*c4 - c2*c3 + c3*d2 - c4*d1)/(c3*d4 - c4*d3)

c1 = cl(clInds,1); c2 = cl(clInds,2); c3 = cl(clInds,3); c4 = cl(clInds,4);
d1 = ll(:,1);      d2 = ll(:,2);      d3 = ll(:,3);      d4 = ll(:,4);

s1 = -(c1*d4' - c2*d3' - ones(length(clInds),1)*(d1.*d4)' + ones(length(clInds),1)*(d2.*d3)')...
    ./(c3*d4' - c4*d3');
s2 = -((c1.*c4)*ones(1,size(ll,1)) - (c2.*c3)*ones(1,size(ll,1)) + c3*d2' - c4*d1')...
    ./(c3*d4' - c4*d3');

% verify the solution
%max(max(cl(clInds,1)*ones(1,size(s1,2))+cl(clInds,3)*ones(1,size(s1,2)).*s1-...
%    (ones(size(s1,1),1)*ll(:,1)'+ones(size(s1,1),1)*ll(:,3)'.*s2)))

% so now I have all of the possible interestions for the 180 lines. A lot
% of these interestions will have s1 values that are outside [0,1], or s2
% values that are less than 0
% for each of the lidar lines
distances=12*ones(LIDAR_NUM_SAMPLES,1);
for i=1:1:size(s1,2)
    validInds=1:size(s1,1);
    validInds((s1(validInds,i)<0)+(s1(validInds,i)>1)>0)=[];
    validInds((s2(validInds,i)<0))=[];
    distances(i)=min([distances(i),s2(validInds,i)']);
end
% plot the results
if 0
figure(2); subplot(1,2,1); cla; hold on;
title('Course and LIDAR Sensor Readings');

% for i=1:1:size(ll,1)
%     plot([ll(i,1),ll(i,1)+ll(i,3)],[ll(i,2),ll(i,2)+ll(i,4)],'b-');
% end
%for i=1:1:size(cl,1)
%    plot([cl(i,1),cl(i,1)+cl(i,3)],[cl(i,2),cl(i,2)+cl(i,4)],'k-');
%end
plot(cl(:,1),cl(:,2),'k.');

%for i=1:1:size(ll,1)
    plot([ll(:,1),ll(:,1)+distances.*ll(:,3)],[ll(:,2),ll(:,2)+distances.*ll(:,4)],'g.');
    plot(ll(1,1),ll(1,2),'b.');
%end
end
obj.sensorData{1}=[angles'-obj.orientation,distances];

% DELTA POSITION AND ORIENTATION
obj.sensorData{2}=obj.intertialSensorValue;

end

