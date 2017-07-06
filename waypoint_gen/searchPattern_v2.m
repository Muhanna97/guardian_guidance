% Search Pattern Generator
% Richard Arthurs
% May 6, 2017

% This algorithm is very simple, will simply chop the last pass, resulting
% in most passes having the desired track width, but the tops and bottoms
% being cut off (narrower or wider) than desired

clf
close all
clear all

%% Functions
m2geo = @(metres) (metres/11.1)*0.0001; % convert meters to decimal degrees https://gis.stackexchange.com/questions/8650/measuring-accuracy-of-latitude-and-longitude
geo2m = @(geo) (geo/0.0001)*11.1; %convert decimal degrees to meters

%%
% Search area coordinates
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
trackWidth = 20; % distance between passes (metres)
numSteps = round(abs((p2.y-p1.y)/m2geo(trackWidth)));

direction = -1; % -1 = flying SOUTH

for i = 1:numSteps
    angWest = atan((p2.y-p1.y)/(p2.x-p1.x));
    westSide(i,1) = -m2geo(trackWidth)*i*cos(angWest) - abs(p1.x);  % referenced to p1.x so need to offset by it
    westSide(i,2) = direction*m2geo(trackWidth)*i*sin(angWest) + p1.y; 
    
    angEast = atan((p4.y-p3.y)/(p4.x-p3.x));
    eastSide(i,1) = m2geo(trackWidth)*i*cos(angEast) - abs(p3.x);  
    eastSide(i,2) = -direction*m2geo(trackWidth)*i*sin(angEast) + p3.y; 

end

eastSide = flipud(eastSide); % flip east side so east-west pairs are collated

% adjust points so they fit in boundaries (happens because of numSteps must be integer)
if direction == -1 && westSide(numSteps,2) < p2.y
    westSide(numSteps,2) = p2.y;
end

westSide = vertcat([p1.x,p1.y],westSide); % include p1 (origin in the list of west side points)
eastSide = vertcat(eastSide,[p3.x,p3.y]); % include p3 (origin in the list of east side points)

%% Plot the survey waypoints
for i = 1:numSteps + 1
    plot(westSide(i,1),westSide(i,2),'b--o')
    plot(eastSide(i,1),eastSide(i,2),'b--o')
end
plot(westSide(1,1),westSide(1,2),'r--x'); % plot the origin in red

%% Check that northeast side is still in track width
if abs(eastSide(1,2) - p4.y) > m2geo(trackWidth)
    fprintf('Northeast is out of track width. Will not receive full coverage. Distance: %f \r\n',geo2m(abs(eastSide(1,2) - p4.y)))
end
