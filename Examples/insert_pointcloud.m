clc
clear
close all

% The occupancyMap3D object stores obstacles in 3-D space, using sensor
% observations to map an environment. Create a map and add points from a
% point cloud to identify obstacles. Then inflate the obstacles in the map
% to ensure safe operating space around obstacles.

% Create an occupancyMap3D object with a map resolution of 10 cells/meter.
map3D = occupancyMap3D(10);
% Define a set of 3-D points as an observation from a pose [x y z qw qx qy qz]. This pose is for the sensor that observes these points and is centered on the origin. Define two sets of points to insert multiple observations.
pose = [0 0 0 1 0 0 0];
points = repmat((0:0.25:2)', 1, 3);
points2 = [(0:0.25:2)' (2:-0.25:0)' (0:0.25:2)'];
maxRange = 5;
% Insert the first set of points using insertPointCloud. The function uses
% the sensor pose and the given points to insert observations into the map.
% The colors displayed correlate to the height of the point only for
% illustrative purposes.
insertPointCloud(map3D,pose,points2,maxRange)
figure
show(map3D)
% Insert the second set of points. The ray between the sensor pose (origin)
% and these points overlap points from the previous insertion. Therefore,
% the free space between the sensor and the new points are updated and 
% marked as free space.
insertPointCloud(map3D,pose,points,maxRange)
figure
show(map3D)

% Inflate the map to add a buffer zone for safe operation around obstacles. Define the vehicle radius and safety distance and use the sum of these values to define the inflation radius for the map.
vehicleRadius = 0.2;
safetyRadius = 0.3;
inflationRadius = vehicleRadius + safetyRadius;
inflateMap(map3D, inflationRadius)
figure
show(map3D)