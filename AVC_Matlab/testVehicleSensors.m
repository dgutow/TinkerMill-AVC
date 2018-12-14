function [] = testController()
clear all;
count=0;
%while count<100
    rng(count);
    count=count+1;
    create_track();

vehicle = Vehicle();

% test the sensor calculations
load course.mat;
METERS_PER_FOOT=0.3048;
if 0
    for i=1:1:size(pathPoints,1)
        vehicle.position=pathPoints(i,:)+sin(i/10)*pathNormal(i,:);
        vehicle.orientation=i*pi/234;
        vehicle.getSensors();
    end
end

if 0METERS_PER_FOOT=0.3048;

% test the time update
vehicle.position=[0,0];
vehicle.orientation=pi/2;
vehicle.speed=0;
vehicle.timeStep=1/10;
for i=1:1:100
    vehicle=vehicle.timeUpdate([i/10;pi/i]);
    vehicle.getSensors();
end
end
video=1;
if video
    v = VideoWriter('MapLocatilizationSearch','MPEG-4');
    open(v);
end
% test out some control
tic;
controller=Controller(8);
vehicle.position=pathPoints(1,:);
controller.estimPosition=pathPoints(1,:)';
controller.estimVelocity=[0;0];
vehicle.orientation=pi/5;
controller.estimOrientation=pi/5;
controller.particles = [pathPoints(1,:)';pi/5];
vehicle.speed=0;
vehicle.timeStep=1/10;
controller.MAP_RESOLUTION=.01;
controller.map=zeros(ceil(max(wallLines(:,1)+10))*100,ceil(max(wallLines(:,2)+10))*100);
controller = fillMap(controller, wallLines);
controller.show=1;
curPathPoint=1;
res(count)=0;

controller.map=[];
load temp.mat;
% inds = find(mapEmpty==0);
% mapEmpty=-(mapEmpty-700);
% mapEmpty(inds)=0;
% mapEmpty=sparse(mapEmpty);
% inds = find(mapFull==0);
% mapFull=-(mapFull-700);
% mapFull(inds)=0;
% mapFull=sparse(mapFull);
controller.mapEmpty=mapEmpty;
controller.mapFull=mapFull;
clear mapEmpty mapFull;
        
for i=1:1:600
    vehicle=vehicle.getSensorData();
    if (mod(i,1)==0)&&(i>1)
        [command,controller,vehicle]=controller.calcCommand(vehicle);
    else
        command=[0,0];
    end
    res(count)=res(count)+norm(controller.particles(1:2,1)-vehicle.position');
    if min(vehicle.sensorData{1}(:,2))<METERS_PER_FOOT
        break;
    end
    plotState(vehicle, command, wallLines,i==1);
    
    % update the currentPathPoint if the position-the current pathPoint is 
    
    while (1)
        if curPathPoint==size(pathPoints,1)
            break;
        end
        nextPathPoint=mod(curPathPoint,size(pathPoints,1))+1;
        curPathDir = [pathPoints(nextPathPoint,1)-pathPoints(curPathPoint,1);
            pathPoints(nextPathPoint,2)-pathPoints(curPathPoint,2)];
        curOffset = [vehicle.position(1)-pathPoints(curPathPoint,1);
            vehicle.position(2)-pathPoints(curPathPoint,2)];
        dotProd=curPathDir'*curOffset;
        if (dotProd>0)*(dotProd>norm(curPathDir)^2)
            curPathPoint=mod(curPathPoint,size(pathPoints,1))+1;
        else
            break;
        end
    end
    if curPathPoint==size(pathPoints,1)
        break;
    end
    
    vehicle=vehicle.timeUpdate(command);
    
    if video
        frame = getframe(figure(3));
        writeVideo(v,frame);
    end
end
disp(['Stopped Run at position ',num2str(curPathPoint),' out of ',num2str(size(pathPoints,1)),' after ',num2str(i),' timesteps']);
%res(count)=curPathPoint;
if i<200
    i;
end
%end
if video
    close(v);
end
toc
end

function plotState(vehicle, command, wallLines,reset)
METERS_PER_FOOT=0.3048;
% our goal here is two have two plots side by side.
% 1. the overall course
% 2. the close up of the current area
persistent ax1 ax2 lr1 rect1 lr2 rect2 dir1 dir2;
if reset
    figure(2); 
    ax1=subplot(1,3,1); cla; hold on;
    ax2=subplot(1,3,2); cla; hold on;
    title(ax1,'Course and Sensor Readings');
    title(ax2,'Close up');
    for i=1:1:size(wallLines,1)
        plot(ax1,[wallLines(i,1),wallLines(i,1)+wallLines(i,3)],[wallLines(i,2),wallLines(i,2)+wallLines(i,4)],'k-');
    end
    for i=1:1:size(wallLines,1)
        plot(ax2,[wallLines(i,1),wallLines(i,1)+wallLines(i,3)],[wallLines(i,2),wallLines(i,2)+wallLines(i,4)],'k-');
    end
    set(ax2,'xticklabel','')
    set(ax2,'yticklabel','')
else
    delete (lr1);
    %delete (rect1);
    delete (lr2);
    %delete (rect2);
    %delete(dir1);
    %delete(dir2);
end

sd=vehicle.sensorData{1};
lr1 = plot(ax1,vehicle.position(1)+sd(:,2).*cos(sd(:,1)+vehicle.orientation),vehicle.position(2)+sd(:,2).*sin(sd(:,1)+vehicle.orientation),'g.');
rect1=plot(ax1,vehicle.position(1),vehicle.position(2),'b.');
%dir1=plot(ax1,vehicle.position(1)+[0,cos(vehicle.lowpassOrientation)],vehicle.position(2)+[0,sin(vehicle.lowpassOrientation)],'b-');
lr2 = plot(ax2,vehicle.position(1)+sd(:,2).*cos(sd(:,1)+vehicle.orientation),vehicle.position(2)+sd(:,2).*sin(sd(:,1)+vehicle.orientation),'g.');
rect2=plot(ax2,vehicle.position(1),vehicle.position(2),'b.');
%dir2=plot(ax2,vehicle.position(1)+[0,cos(vehicle.lowpassOrientation)],vehicle.position(2)+[0,sin(vehicle.lowpassOrientation)],'b-');

xlim(ax2,vehicle.position(1)+13*[-1,1]);
ylim(ax2,vehicle.position(2)+13*[-1,1]);
drawnow;
pause(0.01);


end

function obj = fillMap(obj, wallLines) 
    % assessing the state largely means checking the congruence of hte
    % LIDAR and map information. If the lidar lines pass through a map
    % point then the assess value goes up by 1.
    
    % these are the lines in the body frame
    for i=1:1:size(wallLines,1)
        % so, we know what line we are dealing with. Go from start to stop
        % checking the map elements that we pass through
        curPoint = wallLines(i,[1,2])';
        curAdd=0;
        while (curAdd<1)
            % set this square
            obj.map(floor(curPoint(1)/obj.MAP_RESOLUTION),floor(curPoint(2)/obj.MAP_RESOLUTION))=1;
            % update the point to move it to a new grid
            % the extra length needed to cross a horiz/vertical line
            hChangeLength = 0;
            vChangeLength = 0;
            % now calculate them
            if (wallLines(i,4)>0)
                % we are moving up
                hChangeLength = (ceil(curPoint(2)/obj.MAP_RESOLUTION)-curPoint(2)/obj.MAP_RESOLUTION)*obj.MAP_RESOLUTION/wallLines(i,4);
            elseif (wallLines(i,4)<0)
                hChangeLength = (curPoint(2)/obj.MAP_RESOLUTION-floor(curPoint(2)/obj.MAP_RESOLUTION))*obj.MAP_RESOLUTION/-wallLines(i,4);
            else
                hChangeLength=inf;
            end                
            if (wallLines(i,3)>0)
                % we are moving right
                vChangeLength = (ceil(curPoint(1)/obj.MAP_RESOLUTION)-curPoint(1)/obj.MAP_RESOLUTION)*obj.MAP_RESOLUTION/wallLines(i,3);
            elseif (wallLines(i,3)<0)
                vChangeLength = (curPoint(1)/obj.MAP_RESOLUTION-floor(curPoint(1)/obj.MAP_RESOLUTION))*obj.MAP_RESOLUTION/(-wallLines(i,3));
            else
                vChangeLength=inf;
            end   
            addLength=min([hChangeLength,vChangeLength])+1E-6;
            % move the currentpoint along the line
            if isinf(addLength)
                addLength;
            end
            curPoint = curPoint+addLength*wallLines(i,[3,4])';
            curAdd = curAdd+addLength;
        end
    end
end