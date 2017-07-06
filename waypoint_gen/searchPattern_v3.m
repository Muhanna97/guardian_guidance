% Search Pattern Generator
% Richard Arthurs
% May 6, 2017

% p1 (start) p4
% p2         p3 (end)

clf
close all
clear all

%% Functions
m2geo = @(metres) (metres/11.1)*0.0001; % convert meters to decimal degrees https://gis.stackexchange.com/questions/8650/measuring-accuracy-of-latitude-and-longitude
geo2m = @(geo) (geo/0.0001)*11.1; %convert decimal degrees to meters
haversine_a = @(lat1,long1,lat2,long2) sin(abs(lat2-lat1)/2)^2 + cos(lat1) * cos(lat2) * sin(abs(long2-long1)/2)^2;
haversine = @(lat1,long1,lat2,long2) (2 * atan2(sqrt(haversine_a(lat1,long1,lat2,long2)),sqrt(1-haversine_a(lat1,long1,lat2,long2))))*6378.137; % output = km, input = RADIANS
%%
% Search area coordinates lat = y, long = x
p1.x = -122.7963030;
p1.y = 49.1294287;
p2.x = -122.7963138;
p2.y = 49.1285792;
p3.x = -122.7907777;
p3.y = 49.1285230;
p4.x = -122.7906597;
p4.y = 49.1295129;
          
x = [p1.x,p2.x,p3.x,p4.x];
y = [p1.y,p2.y,p3.y,p4.y];

fill(x,y,'w') % plot the area as a polygon
hold on 

% p1 and p2 are oriented north/south of EO, as are p3, p4
%% determine number of passes
% to ensure the track width is held, at a minimum. Less efficient but gives 100% coverage
trackWidth = 20; % distance between passes (metres)
numSteps = max(abs((p2.y-p1.y)/m2geo(trackWidth)),abs((p4.y-p3.y)/m2geo(trackWidth)));
numSteps = ceil(numSteps); % next highest integer

westStep = abs((p2.y-p1.y)/numSteps); % step size in geo units
eastStep = abs((p4.y-p3.y)/numSteps);

direction = -1; % -1 = flying SOUTH - not fully implemented

for i = 1:numSteps
    angWest = atan((p2.y-p1.y)/(p2.x-p1.x));
    westSide(i,1) = -westStep*i*cos(angWest) - abs(p1.x);  % referenced to p1.x so need to offset by it
    westSide(i,2) = direction*westStep*i*sin(angWest) + p1.y; 

    angEast = atan((p4.y-p3.y)/(p4.x-p3.x));
    eastSide(i,1) = eastStep*i*cos(angEast) - abs(p3.x);  
    eastSide(i,2) = -direction*eastStep*i*sin(angEast) + p3.y; 
end

eastSide = flipud(eastSide); % flip east side so east-west pairs are collated

westSide = vertcat([p1.x,p1.y],westSide); % include p1 (origin in the list of west side points)
eastSide = vertcat(eastSide,[p3.x,p3.y]); % include p3 (origin in the list of east side points)

%% Plot the survey waypoints
for i = 1:numSteps + 1
    plot(westSide(i,1),westSide(i,2),'b--o')
    plot(eastSide(i,1),eastSide(i,2),'b--o')
end
plot(westSide(1,1),westSide(1,2),'r--x'); % plot the origin in red

%% Generate waypoint list
% Sequential order
numPts = numSteps + 1;
eastIndex = 1;
westIndex = 2;
num = 0;

% slightly awkward way to orient the waypoints in the correct order
for i = 1:numPts*2 - 2
    if num == 0 || num == 1 % two east and then two west
        final(i,:) = eastSide(eastIndex,:);
        eastIndex = eastIndex + 1;
        num = num+1;
    else
        final(i,:) = westSide(westIndex,:);
        westIndex = westIndex + 1;
        num = num+1;
    end
    if num == 4
        num = 0;
    end
end

final = vertcat([p1.x,p1.y],final); % insert the first wp at the top
final = vertcat(final,[p3.x,p3.y]); % insert last wp at the bottom

% print out the waypoints in flying order
for i = 1:length(final)
    plot(final(i,1),final(i,2),'r--o')
    pause(.1)
end

%% Calculate Straight Line Distance
% using haversine formula between each waypoint
for i = 1:length(final)-1
    distance(i) = haversine(deg2rad(final(i,2)),deg2rad(final(i,1)),deg2rad(final(i+1,2)),deg2rad(final(i+1,1)));
end
fprintf('Total distance: %f km\r\n',sum(distance))

