function [] = create_track()
%close all;
clear all;

METERS_PER_FOOT=0.3048;

% track parameters
TRACK_X_WIDTH=178*METERS_PER_FOOT;
TRACK_Y_WIDTH=78*METERS_PER_FOOT;
TRACK_WIDTH=16*METERS_PER_FOOT;
NUM_PATH_POINTS=200;

% ramp parameters
RAMP_WIDTH_RANGE=8*METERS_PER_FOOT;
RAMP_WIDTH_MIN=3*METERS_PER_FOOT;
RAMP_LENGTH_RANGE=7*METERS_PER_FOOT;
RAMP_LENGTH_MIN=3*METERS_PER_FOOT;

% Barrel parameters
BARREL_RADIUS=1.5*METERS_PER_FOOT;
BARREL_NUMBER=8;
BARREL_TRACK_LENGTH_RANGE=15*METERS_PER_FOOT;
BARREL_MINIMUM_SEPARATION=2.1*METERS_PER_FOOT;

% this determines how many points we have along the race axis
t=linspace(0,2*pi,NUM_PATH_POINTS);
pathPoints(:,1)=TRACK_X_WIDTH*cos(t)';
pathPoints(:,2)=TRACK_Y_WIDTH*sin(2*t)';

pathPoints(:,1)=pathPoints(:,1)-min(pathPoints(:,1))+10;
pathPoints(:,2)=pathPoints(:,2)-min(pathPoints(:,2))+10;
%figure(1); clf; hold on;
%plot(pathPoints(:,1),pathPoints(:,2),'c.');
%axis equal;
%xlabel('meters');
%ylabel('meters');

% now generate the lines for the points
pathDirs=zeros(NUM_PATH_POINTS,2);
pathDirs(1,:)=pathPoints(2,:)-pathPoints(NUM_PATH_POINTS,:);
pathDirs(NUM_PATH_POINTS,:)=pathPoints(1,:)-pathPoints(NUM_PATH_POINTS-1,:);
for i=2:1:NUM_PATH_POINTS-1
    pathDirs(i,:)=pathPoints(i+1,:)-pathPoints(i-1,:);
end

meanDist=mean(diag(pathDirs*pathDirs').^.5);
maxDist=max(diag(pathDirs*pathDirs').^.5);
% now normalize .... because!
pathDirs=pathDirs./(diag(pathDirs*pathDirs').^.5);
pathNormal=[pathDirs(:,2),-pathDirs(:,1)];

% plot the along path directions
%quiver(pathPoints(:,1),pathPoints(:,2),meanDist*pathDirs(:,1),meanDist*pathDirs(:,2),0,'b');
%quiver(pathPoints(:,1),pathPoints(:,2),meanDist*pathNormal(:,1),meanDist*pathNormal(:,2),0,'r');

%now create the wall points
wallPoints=zeros(2*NUM_PATH_POINTS,2);
for i=1:1:NUM_PATH_POINTS
    wallPoints(2*i-1,:)=pathPoints(i,:)+TRACK_WIDTH/2*pathNormal(i,:);
    wallPoints(2*i,:)  =pathPoints(i,:)-TRACK_WIDTH/2*pathNormal(i,:);
end
%plot(wallPoints(:,1),wallPoints(:,2),'kx');

% figure out which wall points are in the center and kill them
for i=2*NUM_PATH_POINTS:-1:1
    if min(sum((pathPoints-wallPoints(i,:)).^2,2))<(TRACK_WIDTH/2)^2*.99
        wallPoints(i,:)=[];
    end
end
%plot(wallPoints(:,1),wallPoints(:,2),'k.');

% sort the wallPoints into the three wall groups
if maxDist>15*METERS_PER_FOOT
    disp('Too Few points for sorting');
end
wallPoints(:,3)=0;
zeroInds=1:1:size(wallPoints,1);
sortedWallPoints=zeros(0,3);
while ~isempty(zeroInds)
    curVal=round(max(wallPoints(:,3))+1);
    wallPoints(zeroInds(1),3)=curVal;
    curPoint=wallPoints(zeroInds(1),[1,2]);
    sortedWallPoints(end+1,:)=wallPoints(zeroInds(1),:);
    zeroInds(1)=[]; % mark that this point is claimed
    while ~isempty(zeroInds)
        % find the distances to the other unclaimed wallpoints
        [val,ind]=min(sum((wallPoints(zeroInds,[1,2])-curPoint).^2,2).^.5);
        % if this is more than 95% ofthe track width away then it is in a
        % different wall group
        if val>TRACK_WIDTH*.95
            break;
        end
        wallPoints(zeroInds(ind),3)=curVal;
        curPoint=wallPoints(zeroInds(ind),[1,2]);
        sortedWallPoints(end+1,:)=wallPoints(zeroInds(ind),:);
        zeroInds(ind)=[]; % mark that this point is claimed
    end
end

% verify that we have grouped the wall points correctly
%inds=find(sortedWallPoints(:,3)==1);
%plot(sortedWallPoints(inds,1),sortedWallPoints(inds,2),'k.');
%inds=find(sortedWallPoints(:,3)==2);
%plot(sortedWallPoints(inds,1),sortedWallPoints(inds,2),'y.');
%inds=find(sortedWallPoints(:,3)==3);
%plot(sortedWallPoints(inds,1),sortedWallPoints(inds,2),'b.');

% now create the wall lines and define their bounds
% lines are going to be represented as c_i1+c_i2*si for all s in the reals
% intersection of two lines c_11+c_12*s1=c_21+c_22*s2 ->
% c_11-c_21=c_22*s2-c_12*s1
% [1,1]+[2,1]*s1=[0,0]+[1,1]*s2 -> [1,1]=[-2,-1;1,1]*[s1;s2]

% the wall lines variable has 4 parameters [c_i1,c_i2]
% s is always resricted to [0,1]
wallLines=zeros(0,4);
% for each wall section
for i=min(sortedWallPoints(:,3)):1:max(sortedWallPoints(:,3))
    inds=find(round(sortedWallPoints(:,3)-i)==0);
    wallLines=[zeros(length(inds),4);wallLines];
    for j=1:1:length(inds)-1
        wallLines(j,:)=[sortedWallPoints(inds(j),[1,2]),...
            sortedWallPoints(inds(j+1),[1,2])-sortedWallPoints(inds(j),[1,2])];
    end
    j=length(inds);
    wallLines(j,:)=[sortedWallPoints(inds(j),[1,2]),...
        sortedWallPoints(inds(1),[1,2])-sortedWallPoints(inds(j),[1,2])];
end

%for i=1:1:size(wallLines,1)
%    plot([wallLines(i,1),wallLines(i,1)+wallLines(i,3)],[wallLines(i,2),wallLines(i,2)+wallLines(i,4)],'k-');
%end

if 1
% now pick a spot for the ramp
randInd=round(rand(1)*size(pathPoints,1));
rampWidth=RAMP_WIDTH_MIN+rand(1)*RAMP_WIDTH_RANGE;
rampLength=RAMP_LENGTH_MIN+rand(1)*RAMP_LENGTH_RANGE;
rampPoints(1,:)=pathPoints(randInd,:)-...
    pathNormal(randInd,:)*TRACK_WIDTH/2+...
    pathNormal(randInd,:)*rand(1)*(TRACK_WIDTH-rampWidth);
rampPoints(2,:)=rampPoints(1,:)+pathNormal(randInd,:)*rampWidth;
rampPoints([4,3],:)=rampPoints([1,2],:)-pathDirs(randInd,:)*rampLength;

% convert the ramp into 4 lines
wallLines=[[rampPoints(1,:),rampPoints(2,:)-rampPoints(1,:);...
            rampPoints(2,:),rampPoints(3,:)-rampPoints(2,:);...
            rampPoints(3,:),rampPoints(4,:)-rampPoints(3,:);...
            rampPoints(4,:),rampPoints(1,:)-rampPoints(4,:)];...
           wallLines];

% plot the new lines
%for i=1:1:4
%    plot([wallLines(i,1),wallLines(i,1)+wallLines(i,3)],[wallLines(i,2),wallLines(i,2)+wallLines(i,4)],'k-');
%end
end

% now pick a spot for the barrels that is at least 20 feat from the ramp
if 1
while 1
    randInd=round(rand(1)*size(pathPoints,1)+.5);
    if min(sum((rampPoints-pathPoints(randInd,:)).^2,2))^.5>20*METERS_PER_FOOT
        break;
    end
end
% get the center point and rectangle axes
barrelsCenter=pathPoints(randInd,:);
barrelsTrackDir=pathDirs(randInd,:);
barrelsTrackNormal=pathNormal(randInd,:);

barrelPoints=zeros(0,2);
for i=1:1:BARREL_NUMBER
    count=0;
    while 1
        if count>1E5
            break;
        else
            count =count+1;
        end
        newBarrel= barrelsCenter+...
            (rand(1)-.5)*BARREL_TRACK_LENGTH_RANGE*barrelsTrackDir+...
            (rand(1)-.5)*TRACK_WIDTH*barrelsTrackNormal;
        % now check if it is too close to a wallPoint
        if min(sum((wallLines(:,[1,2])-newBarrel).^2,2))^.5<BARREL_RADIUS
            continue;
        end
        % now check if it is too close to an existing barrel
        if ~isempty(barrelPoints)
            if min(sum((barrelPoints-newBarrel).^2,2))^.5<(BARREL_MINIMUM_SEPARATION+2*BARREL_RADIUS)
                continue;
            end
        end
        barrelPoints=[barrelPoints;newBarrel];
        break;
    end
end

% now convert the barrels to wall lines
t=linspace(0,2*pi,30)';
for i=1:1:size(barrelPoints,1)
    curBarrelPoints=barrelPoints(i,:)+BARREL_RADIUS*[cos(t),sin(t)];
    wallLines=[[curBarrelPoints(1:end-1,:),curBarrelPoints(2:end,:)-curBarrelPoints(1:end-1,:)];...
           wallLines];
end
%figure(1);clf; hold on;
%for i=1:1:size(wallLines,1)
%    plot([wallLines(i,1),wallLines(i,1)+wallLines(i,3)],[wallLines(i,2),wallLines(i,2)+wallLines(i,4)],'k-');
%end
end

save course.mat wallLines pathPoints pathDirs pathNormal;
end