% Search Pattern Generator
% Richard Arthurs
% May 6, 2017
clf
close all
clear all
p1.x = -1;
p1.y = 0;
p2.x = 10;
p2.y = -1;
p3.x = 0;
p3.y = 9;
p4.x = 10;
p4.y = 10;
          
plot([p1.x,p2.x],[p1.y,p2.y])
hold on
plot([p1.x,p3.x],[p1.y,p3.y])
plot([p2.x,p4.x],[p2.y,p4.y])
plot([p3.x,p4.x],[p3.y,p4.y])
axis([-2,12,-2,12])

trackWidth = 2; % distance between passes
numSteps = (p3.y-p1.y)/trackWidth;

for i = 1:numSteps
    angLeft = atan((p3.y-p1.y)/(p3.x-p1.x));
    leftSide(i,1) = trackWidth*i*cos(angLeft) - abs(p1.x);  % referenced to p1.x so need to offset by it
    leftSide(i,2) = trackWidth*i*sin(angLeft); 
    plot(leftSide(i,1),leftSide(i,2),'o')

    angRight = atan((p4.y-p2.y)/(p4.x-p2.x));
    rightSide(i,1) = trackWidth*i*cos(angRight) + p2.x;  % referenced to p1.x so need to offset by it
    rightSide(i,2) = trackWidth*i*sin(angRight) - abs(p2.y); 
    plot(rightSide(i,1),rightSide(i,2),'o')
end

          
          