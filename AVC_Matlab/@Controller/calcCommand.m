function [command,obj,vehicle] = calcCommand(obj,vehicle)
%CALCCOMMAND Summary of this function goes here
%   Detailed explanation goes here
sensorData=vehicle.sensorData;
persistent h b;

command=[0,0];
if obj.type==1
    LIDAR_RANGE=12;
    METERS_PER_FOOT=0.3048;
    % this is a simple control strategy. It calculates in 2 degree
    % increments the amount of obstacles ahead (+-44 deg). Based on the overall amount
    % of obstacles it determines what speed to go
    % it then choses the direction with the fewest obstacles
    
    % shift the data so that 0 deg is forward and angles spane -pi to pi.
    sensorData{1}(sensorData{1}(:,1)>pi,1)=sensorData{1}(sensorData{1}(:,1)>pi,1)-2*pi;
    [~,I]=sort(sensorData{1}(:,1));
    sensorData{1}=sensorData{1}(I,:);

    sideComponents=abs(sin((-88:2:88)/180*pi));

    % consider only the readings within +-45%
    inds=find(abs(sensorData{1}(:,1))<=pi/4);
    
    for i=1:1:length(inds)
        tempDists(i,:)=sensorData{1}(inds(i)-44:inds(i)+44,2);
    end
    tempDists(tempDists.*(ones(size(tempDists,1),1)*sideComponents)>METERS_PER_FOOT)=inf;
    forwardDists=min([tempDists],[],2);
    if max(forwardDists<=0)
        i;
    end
    forwardData=[((-44:2:44)/180*pi)',forwardDists];
    % find the forward angles that are tied for the longest distance
    inds=find(forwardData(:,2)==max(forwardData(:,2)));
    % within that find the angle that turns the least
    [~,ind2]=min(abs(forwardData(inds,1)));
    % set our angle command to go to that angle
    command(2)=forwardData(inds(ind2),1);
    % set the speed command according to the forward obstacles
    command(1)=min([13,forwardData(inds(ind2),2)/1]);
    if (min(sensorData{1}(:,2))<METERS_PER_FOOT)
        disp(min(sensorData{1}(:,2)));
    end
    if obj.show
        figure(2); subplot(1,3,3); cla; hold on;
        xlim([-12,12]);
        ylim([-12,12]);
        title('Controller View');
        ylabel('Distance Left (m)');
        xlabel('Distance Forward (m)');

        % convert from lidar readings to cartesian
        LIDARPoints=(sensorData{1}(:,2)*ones(1,2)).*[cos(sensorData{1}(:,1)),sin(sensorData{1}(:,1))];
        plot(LIDARPoints(:,1),LIDARPoints(:,2),'g.');

        %angle = vehicle.lowpassOrientation-vehicle.orientation;
        plot(0,0,'b.','markersize',10);
        %plot([0,cos(angle)],[0,-sin(angle)],'k-');
        plot(command(1)*[0,cos(command(2))],command(1)*[0,sin(command(2))],'b-');
        drawnow;
    end    
elseif obj.type==2
    LIDAR_RANGE=12;
    METERS_PER_FOOT=0.3048;
    TRACK_WIDTH=16*METERS_PER_FOOT;
    % shift the data so that 0 deg is forward and angles spane -pi to pi.
    sensorData{1}(sensorData{1}(:,1)>pi,1)=sensorData{1}(sensorData{1}(:,1)>pi,1)-2*pi;
    
    % consider only the readings within +-45%
    inds=find(abs(sensorData{1}(:,1))<=pi/4);
    forwardData=sensorData{1}(inds,:);
    % sort the data by angle (not strictly necessary)
    [~,I]=sort(forwardData(:,1));
    forwardData=forwardData(I,:);
    % find the forward angles that are tied for the longest distance
    inds=find(forwardData(:,2)==max(forwardData(:,2)));
    % within that find the angle that turns the least
    [~,ind2]=min(abs(forwardData(inds,1)));
    % set our angle command to go to that angle
    command(2)=forwardData(inds(ind2),1);
    % set the speed command according to the forward obstacles
    command(1)=13-sum(max([zeros(size(forwardData,1),1),11-forwardData(:,2)],[],2))/75;
    
    % sort the data by angle
    [~,I]=sort(sensorData{1}(:,1));
    sensorData{1}=sensorData{1}(I,:);
    
    % get the lines from the sensor data
    validLidarInds=find(sensorData{1}(:,2)<LIDAR_RANGE-.1);
    count=sum(validLidarInds(2:end)-validLidarInds(1:end-1)==1)+((validLidarInds(end)-validLidarInds(1))==179);
    
    LIDARPoints=(sensorData{1}(:,2)*ones(1,2)).*[cos(sensorData{1}(:,1)),sin(sensorData{1}(:,1))];
    LIDARLines=zeros(count,4);
    curCount=1;
    for i=1:1:length(validLidarInds)-1
        if validLidarInds(i+1)-validLidarInds(i)~=1
            continue;
        end
        LIDARLines(curCount,:)=[LIDARPoints(validLidarInds(i),:),...
            LIDARPoints(validLidarInds(i+1),:)-LIDARPoints(validLidarInds(i),:)];
        curCount=curCount+1;
    end
    if ((validLidarInds(end)-validLidarInds(1))==179)
        LIDARLines(curCount+1,:)=[LIDARPoints(validLidarInds(end),:),...
            LIDARPoints(validLidarInds(1),:)-LIDARPoints(validLidarInds(end),:)];
    end
    
    % okay, so now we have all of the lines formed by the lidar that aren't
    % at full range. Now, we need to match the lines as being matched. This
    % requires 3 things:
    % 1a. the normal from line 1 from any 0<s1<1 must hit line 2
    % 1b. the normal from line 2 from any 0<s2<1 must hit line 1
    % 2. that normal distance must be within [14.4,17.6] ft
    % 3. the lines must be within 10 deg of parallel.
    
    % first for each line calculate the normalized normal vector and midpoint.
    LIDARPerpLines=LIDARLines;
    LIDARPerpLines(:,1)=LIDARLines(:,1)+.5*LIDARLines(:,3);
    LIDARPerpLines(:,2)=LIDARLines(:,2)+.5*LIDARLines(:,4);
    norms=sum(LIDARLines(:,4).^2+LIDARLines(:,3).^2,2).^.5;
    LIDARPerpLines(:,[3,4])=[-LIDARLines(:,4)./norms,...
        LIDARLines(:,3)./norms];
    
    % now for all of the lines find their intersecton with the courseLines.
    
    % lines are going to be represented as c_i1+c_i2*si for all s in [0,1]
    % intersection of two lines [c(1);c(2)]+[c(3);c(4)]*s1=[d(1);d(2)]+[d(3);d(4)]*s2 ->
    % [c(1)-d(1)]=[-c(3),d(3)]*[s1]
    % [c(2)-d(2)]=[-c(4),d(4)]*[s2] ->
    % s1 = -(c1*d4 - c2*d3 - d1*d4 + d2*d3)/(c3*d4 - c4*d3)
    % s2 = -(c1*c4 - c2*c3 + c3*d2 - c4*d1)/(c3*d4 - c4*d3)
    
    c1 = LIDARLines(:,1);     c2 = LIDARLines(:,2);     c3 = LIDARLines(:,3);     c4 = LIDARLines(:,4);
    d1 = LIDARPerpLines(:,1); d2 = LIDARPerpLines(:,2); d3 = LIDARPerpLines(:,3); d4 = LIDARPerpLines(:,4);
    
    s1 = -(c1*d4' - c2*d3' - ones(length(c1),1)*(d1.*d4)' + ones(length(c1),1)*(d2.*d3)')...
        ./(c3*d4' - c4*d3');
    s2 = -((c1.*c4)*ones(1,length(d1)) - (c2.*c3)*ones(1,length(d1)) + c3*d2' - c4*d1')...
        ./(c3*d4' - c4*d3');
    
    % verify the solution
    %max(max(c1*ones(1,size(s1,2))+c3*ones(1,size(s1,2)).*s1-...
    %    (ones(size(s1,1),1)*d1'+ones(size(s1,1),1)*d3'.*s2)))
    
    % go through all of the lines and figure out which ones satisfy
    % the conditions 1, 2, and 3.
    lineMatches=[];
    s1(s1<0)=0; % check for satisfaction of 1.
    s1(s1>1)=0; % check for satisfaction of 1.
    
    s2=abs(s2);
    s2(abs(s2)<=TRACK_WIDTH-.2)=0;  % check for satisfaction of 2.
    s2(abs(s2)>=TRACK_WIDTH+.2)=0;  % check for satisfaction of 2.
    for i=1:1:size(LIDARPerpLines,1)
        s2(i,i)=0;
    end
    s=s1.*s2;
    for i=1:1:size(LIDARPerpLines,1)
        inds=find(s(:,i)>0);
        for j=1:1:length(inds)
            % check for parallelicity
            angle = acos((LIDARLines(i,[3,4])/norm(LIDARLines(i,[3,4])))...
                *(LIDARLines(inds(j),[3,4])/norm(LIDARLines(inds(j),[3,4])))');
            if angle>pi/2
                angle=angle-pi;
            end
            %abs(angle/pi*180)
            if abs(angle)<pi/180*10
                %lineMatches=[lineMatches;[validLidarInds(i),validLidarInds(inds(j))]];
                lineMatches=[lineMatches;[i,inds(j)]];
            end
        end
    end
    
    % now plot the matches
    lineMatches=sort(lineMatches(:));
    lineMatches(lineMatches(2:end)==lineMatches(1:end-1))=[];
    if obj.show
        figure(2); subplot(1,3,3); cla;

        %if ~isempty(h)
        %    delete(h);
        %    h=[];
        %    delete(b);
        %    b=[];
        %else
            cla; hold on;
            xlim([-12,12]);
            ylim([-12,12]);
            title('Controller View');
            ylabel('Distance Left (m)');
            xlabel('Distance Forward (m)');
        %end
        b=plot(LIDARPoints(:,1),LIDARPoints(:,2),'g.');
        %for i=length(lineMatches):-1:1
        %    h(i)=plot([LIDARLines(lineMatches(i),1),LIDARLines(lineMatches(i),1)+LIDARLines(lineMatches(i),3)],...
        %        [LIDARLines(lineMatches(i),2),LIDARLines(lineMatches(i),2)+LIDARLines(lineMatches(i),4)],'k-','linewidth',2);
        %end
        plot(0,0,'b.','markersize',10);
        plot(command(1)*[0,cos(command(2))],command(1)*[0,sin(command(2))],'b-');
        
        drawnow;
    end    
    % 1. generate the lines from the LIDARPoints
    % 2. Find the lines
    % 2a. chose the most likely slope from the lines
    % 2b. assign lines to the given slope
    % 3. consider shifting to a circle
    
    % 1. generate the lines from the LIDARPoints
    % 2. Find the lines
    % 2a. chose the most likely slope from the lines
    % 2b. assign lines to the given slope
    % 3. consider shifting to a circle
    
%     figure(3); clf; hold on;% plot the angle histogram
%     slopes=atan(LIDARLines(:,3)./LIDARLines(:,4));
%     slopes(slopes<0)=slopes(slopes<0)+pi;
%     norms=sum(LIDARLines(:,3).^2+LIDARLines(:,4).^2,2).^.5;
%     [angleCount,x]=hist(slopes,linspace(0,pi,180));
%     plot(x,angleCount,'b+');
%     [angleCount5]=movingSum(angleCount,5);
%     plot(x,angleCount5,'b-');
%     title('Histogram of wall angles by point count');
%     xlim([0,pi]);
%     ylim([0,length(slopes)]);
%     
%     figure(4); clf; hold on;
%     angleDist=0*angleCount;
%     angleDist(1)=sum(norms(find(slopes<mean(x([1,2])))));
%     angleDist(end)=sum(norms(find(slopes>mean(x([end-1,end])))));
%     for i=2:1:length(x)-1
%         angleDist(i)=sum(norms(find((slopes>mean(x([i-1,i]))).*(slopes<mean(x([i,i+1]))))));
%     end    
%     plot(x,angleDist,'b+');
%     [angleDist5]=movingSum(angleDist,5);
%     plot(x,angleDist5,'b-');
%     ylim([0,60]);
%     if max(angleDist)>60
%         disp('bounds should be higher');
%     end
%     title('Histogram of wall angles by distance');
%     
%     % Find the directions that both 1. have enough points and have 2.
%     % enough distance
%     inds=find((angleCount5*5>10).*(angleDist5*5>10));
%     numClose=[];
%     for i=1:1:length(inds)
%         low=x(i)-5/180*pi;
%         high=x(i)+5/180*pi;
%         if low<0
%             low=low+pi;
%             numClose(i)=length(sum((slopes<high)+(slopes>low)));
%         elseif high>pi
%             high=high-pi;
%             numClose(i)=length(sum((slopes<high)+(slopes>low)));
%         else
%             numClose(i)=length(sum((slopes<high).*(slopes>low)));
%         end
%     end
%     [val,ind]=max(numClose);
%     maxSlope=x(inds(ind));
    
    % now fit points to this line
    
elseif obj.type==3
    % this approach will try the following:
    % 1. Extend current lines to try and find contiguous sections that are
    % either 1. longer than the track width, have a parallel line at track
    % width offset, or continue another line, giving a combined length of more
    % than the track width.
    % 2. Fit 2 ellipsoids to the points
    LARGE_ELLIPSE_MAJOR=(26.17*2)^2;
    LARGE_ELLIPSE_MINOR=(56.69-39.58)^2;
    SMALL_ELLIPSE_MAJOR=(21.31*2)^2;
    SMALL_ELLIPSE_MINOR=(51.82-39.24)^2;
    
    LIDAR_RANGE=12;
    METERS_PER_FOOT=0.3048;
    TRACK_WIDTH=16*METERS_PER_FOOT;
    
    % shift the data so that 0 deg is forward and angles spane -pi to pi.
    sensorData{1}(sensorData{1}(:,1)>pi,1)=sensorData{1}(sensorData{1}(:,1)>pi,1)-2*pi;
    
    % consider only the readings within +-45%
    inds=find(abs(sensorData{1}(:,1))<=pi/4);
    forwardData=sensorData{1}(inds,:);
    % sort the data by angle (not strictly necessary)
    [~,I]=sort(forwardData(:,1));
    forwardData=forwardData(I,:);
    % find the forward angles that are tied for the longest distance
    inds=find(forwardData(:,2)==max(forwardData(:,2)));
    % within that find the angle that turns the least
    [~,ind2]=min(abs(forwardData(inds,1)));
    % set our angle command to go to that angle
    command(2)=forwardData(inds(ind2),1);
    % set the speed command according to the forward obstacles
    command(1)=8-sum(max([zeros(size(forwardData,1),1),11-forwardData(:,2)],[],2))/75;
    
    % sort the data by angle
    [~,I]=sort(sensorData{1}(:,1));
    sensorData{1}=sensorData{1}(I,:);
    
    % convert from lidar readings to cartesian
    LIDARPoints=(sensorData{1}(:,2)*ones(1,2)).*[cos(sensorData{1}(:,1)),sin(sensorData{1}(:,1))];

    % get the lines from the sensor data
    validLidarInds=find(sensorData{1}(:,2)<LIDAR_RANGE-.1);
    validPoints=LIDARPoints(validLidarInds,:);
    count=sum(validLidarInds(2:end)-validLidarInds(1:end-1)==1)+((validLidarInds(end)-validLidarInds(1))==179);

    % get the lines between subsequent points
    LIDARLines=zeros(count,4);
    curCount=1;
    for i=1:1:length(validLidarInds)-1
        if validLidarInds(i+1)-validLidarInds(i)~=1
            continue;
        end
        LIDARLines(curCount,:)=[LIDARPoints(validLidarInds(i),:),...
            LIDARPoints(validLidarInds(i+1),:)-LIDARPoints(validLidarInds(i),:)];
        curCount=curCount+1;
    end
    if ((validLidarInds(end)-validLidarInds(1))==size(sensorData{1},1)-1)
        LIDARLines(curCount+1,:)=[LIDARPoints(validLidarInds(end),:),...
            LIDARPoints(validLidarInds(1),:)-LIDARPoints(validLidarInds(end),:)];
    end
    LIDARNorms=(sum(LIDARLines(:,3).^2+LIDARLines(:,4).^2,2)).^.5;

    % okay first pass, find lines. only consider the lines we already have
    group=zeros(length(validLidarInds),1);
    count=1;
    for i=1:1:size(LIDARLines,1)
        if group(i)>0
            continue;
        end
        direction = LIDARLines(i,[3,4]);
        direction = direction/(direction(1)^2+direction(2)^2)^.5;
        perpDir = [direction(2),-direction(1)];
        offsets = LIDARLines(:,[1,2])*perpDir';
        
        inds=[];
        newInds=find(abs(offsets-offsets(i))<.1);
        while length(newInds)>length(inds)
            inds=newInds;
            % find the best fit for these points
            fit=[LIDARLines(inds,1),ones(length(inds),1)]\LIDARLines(inds,2);
            residuals=LIDARLines(:,2)-LIDARLines(:,1)*fit(1)-fit(2);
            newInds=find(abs(residuals)<.1);
        end
        if (length(inds)>5)&&(sum(LIDARNorms(inds(1:end-1)))>TRACK_WIDTH+.2)
            % go through inds and assess the contiguous groups. Add any
            % contiguous groups that meat the above criterion
            skips=find(abs(sum((LIDARLines(inds(2:end),[1,2])-LIDARLines(inds(1:end-1),[1,2])).^2,2).^.5-LIDARNorms(inds(1:end-1)))>=.05);
            skips=[skips;length(inds)];
            subInds=inds(1:skips(1));
            if (length(subInds)>5)&&(sum(LIDARNorms(subInds(1:end-1)))>TRACK_WIDTH+.2)
                group(subInds)=count;
                count=count+1;
            end
            for j=2:1:length(skips)
                subInds=inds(skips(j-1)+1:skips(j));
                if (length(subInds)>5)&&(sum(LIDARNorms(subInds))>TRACK_WIDTH+.2)
                    group(subInds)=count;
                    count=count+1;
                end
            end
        end
    end
    
    % now consider fitting one of two ellipses
    % we know the ellipses, but not the angle.
    if obj.show
    figure(2); subplot(1,2,2);
    
    if ~isempty(h)
        delete(h);
        h=[];
        delete(b);
        b=[];
    else
        cla; hold on;
        xlim([-12,12]);
        ylim([-12,12]);
        title('Body Frame Wall Lines and LIDAR Points');
        ylabel('Distance Left (m)');
        xlabel('Distance Forward (m)');
    end
    b=plot(LIDARPoints(:,1),LIDARPoints(:,2),'k+');
    h=[];
    for i=1:1:length(group)-1
        if (group(i)==group(i+1))&&(group(i)>0)
            h=[h;plot([LIDARLines(i,1),LIDARLines(i,1)+LIDARLines(i,3)],...
                [LIDARLines(i,2),LIDARLines(i,2)+LIDARLines(i,4)],'b-','linewidth',2)];
        end
    end
    if (group(end)==group(1))&&(group(end)>0)
        h=[h;plot([LIDARLines(end,1),LIDARLines(end,1)+LIDARLines(end,3)],...
            [LIDARLines(end,2),LIDARLines(end,2)+LIDARLines(end,4)],'b-','linewidth',2)];
    end
    drawnow;
    end
elseif obj.type ==4
    LIDAR_RANGE=12;
    METERS_PER_FOOT=0.3048;
    
    % shift the data so that 0 deg is forward and angles spane -pi to pi.
    sensorData{1}(sensorData{1}(:,1)>pi,1)=sensorData{1}(sensorData{1}(:,1)>pi,1)-2*pi;
    
    % consider only the readings within +-45%
    inds=find(abs(sensorData{1}(:,1))<=pi/4);
    forwardData=sensorData{1}(inds,:);
    % sort the data by angle (not strictly necessary)
    [~,I]=sort(forwardData(:,1));
    forwardData=forwardData(I,:);
    % find the forward angles that are tied for the longest distance
    inds=find(forwardData(:,2)==max(forwardData(:,2)));
    % within that find the angle that turns the least
    [~,ind2]=min(abs(forwardData(inds,1)));
    % set our angle command to go to that angle
    command(2)=forwardData(inds(ind2),1);
    
    
    const = 0.4/12*vehicle.speed;
%    vehicle.lowpassOrientation = (1-const)*vehicle.lowpassOrientation+const*vehicle.orientation;
    vehicle.lowpassOrientation = vehicle.lowpassOrientation+1.5*const*vehicle.orientation;
    forwardOffset=vehicle.orientation-vehicle.lowpassOrientation;
    % this is a simple control strategy. It calculates in 2 degree
    % increments the amount of obstacles ahead (+-44 deg). Based on the overall amount
    % of obstacles it determines what speed to go
    % it then choses the direction with the fewest obstacles
    
    % shift the data so that 0 deg is forward and angles span forwardOffset-pi to forwardOffset+pi.
    sensorData{1}(:,1)=sensorData{1}(:,1);%-forwardOffset;
    
    while sum(sensorData{1}(:,1)+command(2)<-pi)
        sensorData{1}(sensorData{1}(:,1)+command(2)<-pi,1)=sensorData{1}(sensorData{1}(:,1)+command(2)<-pi,1)+2*pi;
    end
    while sum(sensorData{1}(:,1)+command(2)>pi)
        sensorData{1}(sensorData{1}(:,1)+command(2)>pi,1)=sensorData{1}(sensorData{1}(:,1)+command(2)>pi,1)-2*pi;
    end
    [~,I]=sort(sensorData{1}(:,1));
    sensorData{1}=sensorData{1}(I,:);

    sideComponents=abs(sin((-88:2:88)/180*pi));

    % consider only the readings within +-45%
    inds=find(abs(sensorData{1}(:,1)-command(2))<=pi/4);
    
    temp=[sensorData{1}(91:180,:);sensorData{1};sensorData{1}(1:90,:)];
    for i=1:1:length(inds)
        tempDists(i,:)=temp(inds(i)+90-44:inds(i)+90+44,2);
    end
    tempDists(tempDists.*(ones(size(tempDists,1),1)*sideComponents)>METERS_PER_FOOT)=inf;
    forwardDists=min([tempDists],[],2);
    if max(forwardDists<=0)
        i;
    end
    forwardData=[sensorData{1}(inds,1),forwardDists];
    % find the forward angles that are tied for the longest distance
    inds=find(forwardData(:,2)==max(forwardData(:,2)));
    % within that find the angle that turns the least
    [~,ind2]=min(abs(forwardData(inds,1)));
    % set our angle command to go to that angle
    command(2)=forwardData(inds(ind2),1);%+forwardOffset;
    % set the speed command according to the forward obstacles
    command(1)=min([13,forwardData(inds(ind2),2)/1]);
    if (min(sensorData{1}(:,2))<METERS_PER_FOOT)
        disp(min(sensorData{1}(:,2)));
    end

    if obj.show
        figure(2); subplot(1,3,3); cla; hold on;
        xlim([-12,12]);
        ylim([-12,12]);
        title('Body Frame Wall Lines and LIDAR Points');
        ylabel('Distance Left (m)');
        xlabel('Distance Forward (m)');

        % convert from lidar readings to cartesian
        LIDARPoints=(sensorData{1}(:,2)*ones(1,2)).*[cos(sensorData{1}(:,1)),sin(sensorData{1}(:,1))];
        plot(LIDARPoints(:,1),LIDARPoints(:,2),'g.');

        %angle = vehicle.lowpassOrientation-vehicle.orientation;
        plot(0,0,'.k','markersize',10);
        %plot([0,cos(angle)],[0,-sin(angle)],'k-');
        plot(command(1)*[0,cos(command(2))],command(1)*[0,sin(command(2))],'b-');
        drawnow;
    end
elseif obj.type==5
    LIDAR_RANGE=12;
    METERS_PER_FOOT=0.3048;
    TRACK_WIDTH=16*METERS_PER_FOOT;
    % shift the data so that 0 deg is forward and angles spane -pi to pi.
    sensorData{1}(sensorData{1}(:,1)>pi,1)=sensorData{1}(sensorData{1}(:,1)>pi,1)-2*pi;
    
    % consider only the readings within +-45%
    inds=find(abs(sensorData{1}(:,1))<=pi/4);
    forwardData=sensorData{1}(inds,:);
    % sort the data by angle (not strictly necessary)
    [~,I]=sort(forwardData(:,1));
    forwardData=forwardData(I,:);
    % find the forward angles that are tied for the longest distance
    inds=find(forwardData(:,2)==max(forwardData(:,2)));
    angleRanges=[];
    inds=[inds;1000];
    inds2=find(inds(2:end)-inds(1:end-1)>1);
    inds2=[0;inds2];
    
    angleRanges=[];
    for i=2:1:length(inds2)
        angleRanges=[round((inds2(i)+inds2(i-1))/2),inds2(i)-inds2(i-1)];
    end
    
    % within that find the angle that turns the least
    [~,ind2]=max(angleRanges(:,2));
    % set our angle command to go to that angle
    command(2)=forwardData(inds(angleRanges(ind2)),1);
    % set the speed command according to the forward obstacles
    command(1)=13-sum(max([zeros(size(forwardData,1),1),11-forwardData(:,2)],[],2))/75;
    if obj.show
        figure(2); subplot(1,3,3); cla; hold on;
        xlim([-12,12]);
        ylim([-12,12]);
        title('Controller View');
        ylabel('Distance Left (m)');
        xlabel('Distance Forward (m)');

        % convert from lidar readings to cartesian
        LIDARPoints=(sensorData{1}(:,2)*ones(1,2)).*[cos(sensorData{1}(:,1)),sin(sensorData{1}(:,1))];
        plot(LIDARPoints(:,1),LIDARPoints(:,2),'g.');

        %angle = vehicle.lowpassOrientation-vehicle.orientation;
        plot(0,0,'b.','markersize',10);
        %plot([0,cos(angle)],[0,-sin(angle)],'k-');
        plot(command(1)*[0,cos(command(2))],command(1)*[0,sin(command(2))],'b-');
        drawnow;
    end
elseif obj.type==6
    LIDAR_RANGE=12;
    METERS_PER_FOOT=0.3048;
    
    command = calcLongCenteredObstacleAvoidance(sensorData,vehicle);
    
    particles = obj.particles;
    oldParticles = obj.particles;
    velocity = obj.estimVelocity;
    NUM_PERTURBS=30;
    for i=1:1:size(particles,2)
        if (size(particles,2)>200)
            break;
        end
        % only change the location. the orientation comes from the
        % direction travelled
        particles = [particles,obj.particles(:,i)+diag([1,1,.03])*(rand(3,NUM_PERTURBS)-.5)];
        oldParticles = [oldParticles,obj.particles(:,i)+zeros(3,NUM_PERTURBS)];
        velocity = [velocity,velocity(:,i)+zeros(2,NUM_PERTURBS)];
    end
    
    % update our location estimate(s) using dead reckoning
    if ~isempty(vehicle.intertialSensorValue)
        for i=1:1:size(particles,2)
            particles(3,i) = particles(3,i)+vehicle.timeStep*vehicle.intertialSensorValue(1);
            velocity(:,i) = velocity(:,i) + vehicle.timeStep*rot(particles(3,i))*vehicle.intertialSensorValue(2:3)';
            particles(1:2,i) = particles(1:2,i) + vehicle.timeStep*velocity(:,i);
            %temp =(particles(1:2,i)-oldParticles(1:2,i));
            %particles(3,i) = acos(temp(1)/norm(temp));
            % now make things fit
            %particles(3,i) = particles(3,i)+-2*particles(3,i)*(temp(2)<0);
            %velocity(:,i) = temp/vehicle.timeStep;
        end
    end

    % filter based on velocity
    speed = diag(velocity'*velocity).^.5;
    inds=find(abs(speed-vehicle.speed)>.5);
    velocity(:,inds)=[];
    particles(:,inds)=[];
    oldParticles(:,inds)=[];
%    obj.estimVelocity = (obj.particles(1:2,:) - oldPosition)/vehicle.timeStep;

    % these are the lines in the body frame
    LIDARLines= getLidarLines(obj,vehicle);
    for i=1:1:size(particles,2)
        value(i)=assessState(obj,particles(1:2,i),particles(3,i),vehicle,LIDARLines);
    end
    [~,I]=sort(value,'descend');
    maxValue = max(value); % find the maximum value
    inds=find(value==maxValue);
    obj.particles=particles(:,inds);
    obj.estimVelocity=velocity(:,inds);

    %obj.estimVelocity = (obj.particles(1:2,:) - oldPosition)/vehicle.timeStep;
    % now update the map
    if obj.show
        figure(2); subplot(1,3,3); cla; hold on;
        xlim([-12,12]);
        ylim([-12,12]);
        title('Controller View');
        ylabel('Distance Left (m)');
        xlabel('Distance Forward (m)');

        % convert from lidar readings to cartesian
        LIDARPoints=(sensorData{1}(:,2)*ones(1,2)).*[cos(sensorData{1}(:,1)),sin(sensorData{1}(:,1))];
        plot(LIDARPoints(:,1),LIDARPoints(:,2),'g.');

        %angle = vehicle.lowpassOrientation-vehicle.orientation;
        plot(0,0,'b.','markersize',10);
        %plot([0,cos(angle)],[0,-sin(angle)],'k-');
        plot(command(1)*[0,cos(command(2))],command(1)*[0,sin(command(2))],'b-');
        drawnow;

        figure(3);% clf; hold on;
        persistent h6 r g;
        if ~isempty(h6)
            delete(h6);
            delete(r);
            delete(g);
        else
            figure(3); clf; hold on;
            ind = find(obj.map);
            plot(.01*(mod(ind-1,size(obj.map,1))+1),floor(ind/size(obj.map,1))*.01,'k.');
        end
        for i=1:1:size(obj.particles,2)
            h6(i)=plot(obj.particles(1,i),obj.particles(2,i),'b.');
        end
        r = plot(vehicle.position(1),vehicle.position(2),'ro');
        
        % convert to global frame
        LIDARPoints(:,[1,2])=(rot(obj.particles(3,1) )*LIDARPoints(:,[1,2])')';
        LIDARPoints(:,1)=LIDARPoints(:,1)+obj.particles(1,1);
        LIDARPoints(:,2)=LIDARPoints(:,2)+obj.particles(2,1);
        g = plot(LIDARPoints(:,1),LIDARPoints(:,2),'g.');
        ylim([0,70]);
        title(['Num Overlapping Points: ',num2str(maxValue),' Num Particles: ',num2str(size(obj.particles,2))]);
        drawnow;

    end
elseif obj.type==7
    LIDAR_RANGE=12;
    METERS_PER_FOOT=0.3048;
    
    COARSE_RESOLUTION=1;
    MEDIUM_RESOLUTION=.1;
    FINE_RESOLUTION=.01;
    
    if isempty(obj.map10)
        obj.map10 = zeros(floor(size(obj.map)/10));
        obj.map100 = zeros(floor(size(obj.map10)/10));
        for i=1:1:size(obj.map10,1)
            ind1=max([1,(i)*10-5]):min([size(obj.map,1),(i)*10+4]);
            for j=1:1:size(obj.map10,2)
                ind2=max([1,(j)*10-5]):min([size(obj.map,2),(j)*10+4]);
                obj.map10(i,j)=max(max(obj.map(ind1,ind2)));
            end
        end
        for i=1:1:size(obj.map100,1)
            ind1=max([1,(i)*10-5]):min([size(obj.map10,1),(i)*10+4]);
            for j=1:1:size(obj.map100,2)
                ind2=max([1,(j)*10-5]):min([size(obj.map10,2),(j)*10+4]);
                obj.map100(i,j)=max(max(obj.map10(ind1,ind2)));
            end
        end
        obj.map=sparse(obj.map);
        obj.map10=sparse(obj.map10);
        obj.map100=sparse(obj.map100);
    end
    
    command = calcLongCenteredObstacleAvoidance(sensorData,vehicle);
    
    particles = obj.particles;
    oldParticles = obj.particles;
    velocity = obj.estimVelocity;

    for i=1:1:5
        for j=1:1:5
            for k=1:1:5
                particles=[particles,particles(:,1)+COARSE_RESOLUTION*[(i-3);j-3;0.0873*(k-3)]];
                oldParticles=[oldParticles,oldParticles(:,1)];
                velocity=[velocity,velocity(:,1)];
           end
        end
    end
    
    % update our location estimate(s) using dead reckoning
    if ~isempty(vehicle.intertialSensorValue)
        for i=1:1:size(particles,2)
            particles(3,i) = particles(3,i)+vehicle.timeStep*vehicle.intertialSensorValue(1);
            velocity(:,i) = velocity(:,i) + vehicle.timeStep*rot(particles(3,i))*vehicle.intertialSensorValue(2:3)';
            particles(1:2,i) = particles(1:2,i) + vehicle.timeStep*velocity(:,i);
            temp =(particles(1:2,i)-oldParticles(1:2,i));
            % now make things fit what just happened
            if 0&&norm(temp)>0
                particles(3,i) = acos(temp(1)/norm(temp));
                particles(3,i) = particles(3,i)+-2*particles(3,i)*(temp(2)<0);
                velocity(:,i) = temp/vehicle.timeStep;
            end
        end
    end
    %particles(1:2,:)=round(particles(1:2,:));

    % now run the coarse search from where we are
    LIDARPoints= getLidarLines(obj,vehicle);
    LIDARPoints=LIDARPoints(:,[1,2]);

    % assess the coarse maps
    % sort for more efficient computation
%     [~,I]=sort(particles(3,:));
%     particles=particles(:,I);
%     oldParticles=oldParticles(:,I);
%     velocity=velocity(:,I);

    value=zeros(size(particles,2),1);
    oldAngle=-10;
    curPoints = LIDARPoints;
    for i=1:1:size(particles,2)
        if abs(oldAngle-particles(3,i))>1E-8
            curPoints = (rot(particles(3,i))*LIDARPoints')';
            oldAngle=particles(3,i);
        end
        value(i) = assessStateQuick(obj.map100, curPoints, particles(1:2,i), [], COARSE_RESOLUTION);
    end
    % cull based on the coarse fit
    maxValue=max(value);
    inds=find(value>maxValue*.99);
    velocity=velocity(:,inds);
    particles=particles(:,inds);
    oldParticles=oldParticles(:,inds);
    %disp([particles(:,1),[vehicle.position';vehicle.orientation]]);
    assessStateQuick(obj.map100, LIDARPoints, particles(1:2,1), particles(3,1), COARSE_RESOLUTION, 1);

    % perturb at the medium level
    for n=size(particles,2):-1:1
        for i=1:1:10
            for j=1:1:10
                for k=1:1:10
                    particles=[particles,particles(:,n)+MEDIUM_RESOLUTION*[i-5;j-5;0.0873*(k-5)]];
                    oldParticles=[oldParticles,oldParticles(:,n)];
                    velocity=[velocity,velocity(:,n)];
                end
            end
        end    
    end
    % sort for more efficient computation
    [~,I]=sort(particles(3,:));
    particles=particles(:,I);
    oldParticles=oldParticles(:,I);
    velocity=velocity(:,I);
    % assess the medium maps
    value=zeros(size(particles,2),1);
    oldAngle=-10;
    curPoints = [];
    for i=1:1:size(particles,2)
        if abs(oldAngle-particles(3,i))>1E-8
            curPoints = (rot(particles(3,i))*LIDARPoints')';
            oldAngle=particles(3,i);
        end
        %disp(i);
        value(i) = assessStateQuick(obj.map10, curPoints, particles(1:2,i), [], MEDIUM_RESOLUTION);
    end
    % cull based on the medium fit
    maxValue=max(value);
    inds=find(value>maxValue*.99);
    velocity=mean(velocity(:,inds),2);
    particles=mean(particles(:,inds),2);
    oldParticles=mean(oldParticles(:,inds),2);
    assessStateQuick(obj.map10, LIDARPoints, particles(1:2,1), particles(3,1), MEDIUM_RESOLUTION,1);
    
%     % perturb at the fine level
%     for n=size(particles,2):-1:1
%         for i=1:1:10
%             for j=1:1:10
%                 for k=1:1:10
%                     particles=[particles,particles(:,n)+FINE_RESOLUTION*[i-5;j-5;0.0873*(k-5)]];
%                     oldParticles=[oldParticles,oldParticles(:,n)];
%                     velocity=[velocity,velocity(:,n)];
%                 end
%             end
%         end    
%     end
%     % sort for more efficient computation
%     [~,I]=sort(particles(3,:));
%     particles=particles(:,I);
%     oldParticles=oldParticles(:,I);
%     velocity=velocity(:,I);
%     % assess the medium maps
%     value=zeros(size(particles,2),1);
%     oldAngle=-10;
%     curPoints = [];
%     for i=1:1:size(particles,2)
%         if abs(oldAngle-particles(3,i))>1E-8
%             curPoints = (rot(particles(3,i))*LIDARPoints')';
%             oldAngle=particles(3,i);
%         end
%         %disp(i);
%         value(i) = assessStateQuick(obj.map, curPoints, particles(1:2,i), [], FINE_RESOLUTION);
%     end
%     % cull based on the medium fit
%     maxValue=max(value);
%     inds=find(value>maxValue*.99);
%     velocity=mean(velocity(:,inds),2);
%     particles=mean(particles(:,inds),2);
%     oldParticles=mean(oldParticles(:,inds),2);
% 
%     assessStateQuick(obj.map, fineLidarMap, particles(1:2,1), particles(3,1), FINE_RESOLUTION, 1);
%     
    obj.particles=particles;
    obj.estimVelocity=velocity;

    %obj.estimVelocity = (obj.particles(1:2,:) - oldPosition)/vehicle.timeStep;
    % now update the map
    if obj.show
        figure(2); subplot(1,3,3); cla; hold on;
        xlim([-12,12]);
        ylim([-12,12]);
        title('Controller View');
        ylabel('Distance Left (m)');
        xlabel('Distance Forward (m)');

        % convert from lidar readings to cartesian
        LIDARPoints=(sensorData{1}(:,2)*ones(1,2)).*[cos(sensorData{1}(:,1)),sin(sensorData{1}(:,1))];
        plot(LIDARPoints(:,1),LIDARPoints(:,2),'g.');

        %angle = vehicle.lowpassOrientation-vehicle.orientation;
        plot(0,0,'b.','markersize',10);
        %plot([0,cos(angle)],[0,-sin(angle)],'k-');
        plot(command(1)*[0,cos(command(2))],command(1)*[0,sin(command(2))],'b-');
        drawnow;

        figure(3);% clf; hold on;
        persistent h7 r7 g7;
        if ~isempty(h7)
            delete(h7);
            delete(r7);
            delete(g7);
        else
            figure(3); clf; hold on;
            ind = find(obj.map);
            plot(.01*(mod(ind-1,size(obj.map,1))+1),floor(ind/size(obj.map,1))*.01,'k.');
        end
        for i=1:1:size(obj.particles,2)
            h7(i)=plot(obj.particles(1,i),obj.particles(2,i),'b.');
        end
        r7 = plot(vehicle.position(1),vehicle.position(2),'ro');
        
        % convert to global frame
        LIDARPoints(:,[1,2])=(rot(obj.particles(3,1) )*LIDARPoints(:,[1,2])')';
        LIDARPoints(:,1)=LIDARPoints(:,1)+obj.particles(1,1);
        LIDARPoints(:,2)=LIDARPoints(:,2)+obj.particles(2,1);
        g7 = plot(LIDARPoints(:,1),LIDARPoints(:,2),'g.');
        ylim([0,70]);
        title(['Num Overlapping Points: ',num2str(maxValue),' Num Particles: ',num2str(size(obj.particles,2))]);
        drawnow;

    end
end

end

function [res]=movingSum(n,filterSize)
    if mod(filterSize,2)==0
        filterSize=filterSize+1;
    end
    res=0*n;
    n=[n,n];
    n=n(:);
    for i=filterSize:(filterSize+length(n)/2)
        res(i-(filterSize-1)/2)=sum(n(i-filterSize+1:i));
    end
    res=res/filterSize;
    res(1:(filterSize-1)/2)=res(length(res)-(filterSize-1)/2:length(res)-1);
    res=res(1:length(n)/2);
end

function [grouping]=findLine(LIDARLines)
    slopes=atan(LIDARLines(:,3)./LIDARLines(:,4));
    slopes(slopes<0)=slopes(slopes<0)+pi;
    norms=sum(LIDARLines(:,3).^2+LIDARLines(:,4).^2,2).^.5;
    [angleCount,x]=hist(slopes,linspace(0,pi,180));
    %plot(x,angleCount,'b+');
    [angleCount5]=movingSum(angleCount,5);
    %plot(x,angleCount5,'b-');
    %title('Histogram of wall angles by point count');
    %xlim([0,pi]);
    %ylim([0,length(slopes)]);
    
    %figure(4); clf; hold on;
    angleDist=0*angleCount;
    angleDist(1)=sum(norms(find(slopes<mean(x([1,2])))));
    angleDist(end)=sum(norms(find(slopes>mean(x([end-1,end])))));
    for i=2:1:length(x)-1
        angleDist(i)=sum(norms(find((slopes>mean(x([i-1,i]))).*(slopes<mean(x([i,i+1]))))));
    end    
    %plot(x,angleDist,'b+');
    [angleDist5]=movingSum(angleDist,5);
    %plot(x,angleDist5,'b-');
    %ylim([0,60]);
    %if max(angleDist)>60
    %    disp('bounds should be higher');
    %end
    %title('Histogram of wall angles by distance');
    
    % Find the directions that both 1. have enough points and have 2.
    % enough distance
    possibleLineSlopes=find((angleCount5*5>10).*(angleDist5*5>10));
    numClose=[];
    for i=1:1:length(possibleLineSlopes)
        % find the perpendicular to the current line
        slope=[sin(possibleLineSlopes(i)),cos(possibleLineSlopes(i))];
        perpSlope=[slope(2),-slope(1)];
        % c1+c2*s
        % for the points that go with the LIDARLines, find the component of
        % its start point that is along the perpendicular.
        low=x(possibleLineSlopes(i))-5/180*pi;
        high=x(possibleLineSlopes(i))+5/180*pi;
        if low<0
            low=low+pi;
            lineInds=length(sum((slopes<high)+(slopes>low)));
        elseif high>pi
            high=high-pi;
            lineInds=length(sum((slopes<high)+(slopes>low)));
        else
            lineInds=length(sum((slopes<high).*(slopes>low)));
        end
        perpComps=LIDARLines(lineInds,[1,2])-(LIDARLines(lineInds,[1,2])*perpSlope)*perpSlope;
        perpMagnitudes=(perpComps(:,1).^2+perpComps(:,2).^2).^.5;
        % find the spots where there are at least x points within .2 m.
        % Then assign them and record the b value.
    end
    [val,ind]=max(numClose);
    maxSlope=x(inds(ind));
end

function x = rot(angle) 
    x=[ cos(angle),-sin(angle);
        sin(angle), cos(angle)];
end

function value = assessState(obj, position,orientation, vehicle, LIDARLines) 
    % assessing the state largely means checking the congruence of hte
    % LIDAR and map information. If the lidar lines pass through a map
    % point then the assess value goes up by 1.
    
    % convert to global frame
    LIDARLines(:,[1,2])=(rot(orientation )*LIDARLines(:,[1,2])')';
    LIDARLines(:,[3,4])=(rot(orientation )*LIDARLines(:,[3,4])')';
    LIDARLines(:,1)=LIDARLines(:,1)+position(1);
    LIDARLines(:,2)=LIDARLines(:,2)+position(2);
    
    value = 0;
    oldInds=[0,0];
    for i=1:1:size(LIDARLines,1)
        % so, we know what line we are dealing with. Go from start to stop
        % checking the map elements that we pass through
        curPoint = LIDARLines(i,[1,2])';
        curAdd=0;
        while (curAdd<1)
            % assess this square
            newInds=[floor(curPoint(1)/obj.MAP_RESOLUTION),floor(curPoint(2)/obj.MAP_RESOLUTION)];
            %if obj.map(floor(curPoint(1)/obj.MAP_RESOLUTION),floor(curPoint(2)/obj.MAP_RESOLUTION))
                %disp(obj.map(floor(curPoint(1)/obj.MAP_RESOLUTION),floor(curPoint(2)/obj.MAP_RESOLUTION)));
            %end
            if (sum(newInds==oldInds)<2)&&(obj.map(newInds(1),newInds(2)))
                %disp([i,floor(curPoint(1)/obj.MAP_RESOLUTION),floor(curPoint(2)/obj.MAP_RESOLUTION)]);
                value=value+1;
            end
            oldInds=newInds;
            % update the point to move it to a new grid
            % the extra length needed to cross a horiz/vertical line
            hChangeLength = 0;
            vChangeLength = 0;
            % now calculate them
            if (LIDARLines(i,4)>0)
                % we are moving up
                hChangeLength = (ceil(curPoint(2)/obj.MAP_RESOLUTION)-curPoint(2)/obj.MAP_RESOLUTION)*obj.MAP_RESOLUTION/LIDARLines(i,4);
            else
                hChangeLength = (curPoint(2)/obj.MAP_RESOLUTION-floor(curPoint(2)/obj.MAP_RESOLUTION))*obj.MAP_RESOLUTION/-LIDARLines(i,4);
            end                
            if (LIDARLines(i,3)>0)
                % we are moving right
                vChangeLength = (ceil(curPoint(1)/obj.MAP_RESOLUTION)-curPoint(1)/obj.MAP_RESOLUTION)*obj.MAP_RESOLUTION/LIDARLines(i,3);
            else
                vChangeLength = (curPoint(1)/obj.MAP_RESOLUTION-floor(curPoint(1)/obj.MAP_RESOLUTION))*obj.MAP_RESOLUTION/(-LIDARLines(i,3));
            end     
            addLength=min([hChangeLength,vChangeLength])+1E-6;
            % move the currentpoint along the line
            if isinf(addLength)
                addLength;
            end
            curPoint = curPoint+addLength*LIDARLines(i,[3,4])';
            curAdd = curAdd+addLength;
        end
    end
end

function value = assessStateQuick(courseMap, lidarPoints, position, orientation, resolution, show) 
    % assessing the state largely means checking the congruence of hte
    % LIDAR and map information. If the lidar lines pass through a map
    % point then the assess value goes up by 1.
    if nargin()<6
        show=0;
    end
    % convert to lidarMap to global frame
    if ~isempty(orientation)
        lidarPoints = (rot(orientation)*lidarPoints')';
    end
    offset = position;
    value = 0;
    
    for i=1:1:size(lidarPoints,1)
        value=value+courseMap(round((offset(1)+lidarPoints(i,1))/resolution),...
            round((offset(2)+lidarPoints(i,2))/resolution));
    end
    
    if show
        combMap = full(courseMap);
        oldInd1=0;
        oldInd2=0;
        for i=1:1:size(lidarPoints,1)
            ind1=round((offset(1)+lidarPoints(i,1))/resolution);
            ind2=round((offset(2)+lidarPoints(i,2))/resolution);
            if (ind1==oldInd1)&&(ind2==oldInd2)
                continue;
            end
            oldInd1=ind1;
            oldInd2=ind2;
            combMap(ind1,ind2)=combMap(ind1,ind2)-1;
        end
        rowInds=round(max([1,(offset(1)-13)/resolution]):min([size(combMap,1),(offset(1)+13)/resolution]));
        colInds=round(max([1,(offset(2)-13)/resolution]):min([size(combMap,2),(offset(2)+13)/resolution]));
        combMap(rowInds(1),colInds(1))=2;
        figure(4); surf(combMap(rowInds,colInds),'linestyle','none');
        %figure(4); surf(combMap,'linestyle','none');
        colormap([1,1,1; .8,.8,.8;0,0,0]);
        view(2)
        title(['value: ',num2str(value)]);
        axis tight;
        drawnow;
        pause(0.01);
        
    end
    
end


function LIDARLines= getLidarLines(obj,vehicle)
    LIDAR_RANGE=12;

    sensorData=vehicle.sensorData;
    % sort the data by angle
    [~,I]=sort(sensorData{1}(:,1));
    sensorData{1}=sensorData{1}(I,:);
    
    % convert from lidar readings to cartesian
    LIDARPoints=(sensorData{1}(:,2)*ones(1,2)).*[cos(sensorData{1}(:,1)),sin(sensorData{1}(:,1))];

    % get the lines from the sensor data
    validLidarInds=find(sensorData{1}(:,2)<LIDAR_RANGE-.1);
    validPoints=LIDARPoints(validLidarInds,:);
    count=sum(validLidarInds(2:end)-validLidarInds(1:end-1)==1)+((validLidarInds(end)-validLidarInds(1))==179);

    % get the lines between subsequent points
    LIDARLines=zeros(count,4);
    curCount=1;
    for i=1:1:length(validLidarInds)-1
        if validLidarInds(i+1)-validLidarInds(i)~=1
            continue;
        end
        LIDARLines(curCount,:)=[LIDARPoints(validLidarInds(i),:),...
            LIDARPoints(validLidarInds(i+1),:)-LIDARPoints(validLidarInds(i),:)];
        curCount=curCount+1;
    end
    if ((validLidarInds(end)-validLidarInds(1))==size(sensorData{1},1)-1)
        LIDARLines(end,:)=[LIDARPoints(validLidarInds(end),:),...
            LIDARPoints(validLidarInds(1),:)-LIDARPoints(validLidarInds(end),:)];
    end
end

function command = calcLongCenteredObstacleAvoidance(sensorData,vehicle)
    LIDAR_RANGE=12;
    METERS_PER_FOOT=0.3048;
    
    % shift the data so that 0 deg is forward and angles spane -pi to pi.
    sensorData{1}(sensorData{1}(:,1)>pi,1)=sensorData{1}(sensorData{1}(:,1)>pi,1)-2*pi;
    
    % consider only the readings within +-45%
    inds=find(abs(sensorData{1}(:,1))<=pi/4);
    forwardData=sensorData{1}(inds,:);
    % sort the data by angle (not strictly necessary)
    [~,I]=sort(forwardData(:,1));
    forwardData=forwardData(I,:);
    % find the forward angles that are tied for the longest distance
    inds=find(forwardData(:,2)==max(forwardData(:,2)));
    % within that find the angle that turns the least
    [~,ind2]=min(abs(forwardData(inds,1)));
    % set our angle command to go to that angle
    command(2)=forwardData(inds(ind2),1);
    
    
    const = 0.4/12*vehicle.speed;
%    vehicle.lowpassOrientation = (1-const)*vehicle.lowpassOrientation+const*vehicle.orientation;
    vehicle.lowpassOrientation = vehicle.lowpassOrientation+1.5*const*vehicle.orientation;
    forwardOffset=vehicle.orientation-vehicle.lowpassOrientation;
    % this is a simple control strategy. It calculates in 2 degree
    % increments the amount of obstacles ahead (+-44 deg). Based on the overall amount
    % of obstacles it determines what speed to go
    % it then choses the direction with the fewest obstacles
    
    % shift the data so that 0 deg is forward and angles span forwardOffset-pi to forwardOffset+pi.
    sensorData{1}(:,1)=sensorData{1}(:,1);%-forwardOffset;
    
    while sum(sensorData{1}(:,1)+command(2)<-pi)
        sensorData{1}(sensorData{1}(:,1)+command(2)<-pi,1)=sensorData{1}(sensorData{1}(:,1)+command(2)<-pi,1)+2*pi;
    end
    while sum(sensorData{1}(:,1)+command(2)>pi)
        sensorData{1}(sensorData{1}(:,1)+command(2)>pi,1)=sensorData{1}(sensorData{1}(:,1)+command(2)>pi,1)-2*pi;
    end
    [~,I]=sort(sensorData{1}(:,1));
    sensorData{1}=sensorData{1}(I,:);

    sideComponents=abs(sin((-88:2:88)/180*pi));

    % consider only the readings within +-45%
    inds=find(abs(sensorData{1}(:,1)-command(2))<=pi/4);
    
    temp=[sensorData{1}(91:180,:);sensorData{1};sensorData{1}(1:90,:)];
    for i=1:1:length(inds)
        tempDists(i,:)=temp(inds(i)+90-44:inds(i)+90+44,2);
    end
    tempDists(tempDists.*(ones(size(tempDists,1),1)*sideComponents)>METERS_PER_FOOT)=inf;
    forwardDists=min([tempDists],[],2);
    if max(forwardDists<=0)
        i;
    end
    forwardData=[sensorData{1}(inds,1),forwardDists];
    % find the forward angles that are tied for the longest distance
    inds=find(forwardData(:,2)==max(forwardData(:,2)));
    % within that find the angle that turns the least
    [~,ind2]=min(abs(forwardData(inds,1)));
    % set our angle command to go to that angle
    command(2)=forwardData(inds(ind2),1);%+forwardOffset;
    % set the speed command according to the forward obstacles
    command(1)=min([13,forwardData(inds(ind2),2)/1]);
end

function map = fillMap(map, wallLines, resolution) 
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
            map(floor(curPoint(1)/resolution),floor(curPoint(2)/resolution))=1;
            % update the point to move it to a new grid
            % the extra length needed to cross a horiz/vertical line
            hChangeLength = 0;
            vChangeLength = 0;
            % now calculate them
            if (wallLines(i,4)>0)
                % we are moving up
                hChangeLength = (ceil(curPoint(2)/resolution)-curPoint(2)/resolution)*resolution/wallLines(i,4);
            elseif (wallLines(i,4)<0)
                hChangeLength = (curPoint(2)/resolution-floor(curPoint(2)/resolution))*resolution/-wallLines(i,4);
            else
                hChangeLength=inf;
            end                
            if (wallLines(i,3)>0)
                % we are moving right
                vChangeLength = (ceil(curPoint(1)/resolution)-curPoint(1)/resolution)*resolution/wallLines(i,3);
            elseif (wallLines(i,3)<0)
                vChangeLength = (curPoint(1)/resolution-floor(curPoint(1)/resolution))*resolution/(-wallLines(i,3));
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

function outputMap = rotImage(map,orientation)
    outputMap=0*map;
    rotMat=rot(orientation);
    center=size(outputMap)'/2;
    mapSize=size(outputMap)';
    for i=1:1:size(outputMap,1)
        for j=1:1:size(outputMap,2)
            inds=round(rotMat*([i;j]-center)+center);
            if (inds(1)>mapSize(1))||(inds(1)<1)||(inds(2)>mapSize(2))||(inds(2)<1)
                continue;
            end
            outputMap(i,j) = map(inds(1),inds(2));
        end
    end
end