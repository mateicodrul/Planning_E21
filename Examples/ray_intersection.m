clc
clear
close all

map3D = importOccupancyMap3D("citymap.ot");
inflate(map3D,1)
% or with dot notation
% map3D.inflate(1)

numRays = 10;
angles = linspace(-pi/2,pi/2,numRays);
directions = [cos(angles); sin(angles); zeros(1,numRays)]';
sensorPose = [55 40 1 1 0 0 0];
maxrange = 150;
% Directions do not need to have a norm of 1
[intersectionPts, isOccupied] = rayIntersection(map3D,sensorPose,122*directions,maxrange);
% Elements of isOccupied are:
% 1, if ray meets obstacle in the direction, at a distance smaller
% than maxrange. This implies intersectionPt is occupied
% -1, if ray meets no obstacle in the direction for a distance equal to
% maxrange and intersectionPt is unmapped
% 0, if ray meets no obstacle in the direction for a distance equal to
% maxrange and intersectionPt is free

figure
show(map3D)
hold on
% Vehicle sensor pose
plotTransforms(sensorPose(1:3),sensorPose(4:end),...
               'FrameSize',5,'MeshFilePath','groundvehicle.stl') 
for i = 1:numRays
    % Plot rays
    plot3([sensorPose(1),intersectionPts(i,1)],...
          [sensorPose(2),intersectionPts(i,2)],...
          [sensorPose(3),intersectionPts(i,3)],'-b') 
    if isOccupied(i) == 1
        % Intersection points
        plot3(intersectionPts(i,1),intersectionPts(i,2),intersectionPts(i,3),'*r')
    end
end