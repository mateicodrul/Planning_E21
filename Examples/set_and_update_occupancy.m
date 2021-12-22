clc
clear
close all

map3D = occupancyMap3D(1);
% Create a ground plane and set occupancy values to 0.
[xGround,yGround,zGround] = meshgrid(0:100,0:100,0);
xyzGround = [xGround(:) yGround(:) zGround(:)];
occval = 0;
setOccupancy(map3D,xyzGround,occval)
% Create obstacles in specific world locations of the map.
[xBuilding1,yBuilding1,zBuilding1] = meshgrid(20:30,50:60,0:30);
[xBuilding2,yBuilding2,zBuilding2] = meshgrid(50:60,10:30,0:40);
[xBuilding3,yBuilding3,zBuilding3] = meshgrid(40:60,50:60,0:50);
[xBuilding4,yBuilding4,zBuilding4] = meshgrid(70:80,35:45,0:60);
xyzBuildings = [xBuilding1(:) yBuilding1(:) zBuilding1(:);...
                xBuilding2(:) yBuilding2(:) zBuilding2(:);...
                xBuilding3(:) yBuilding3(:) zBuilding3(:);...
                xBuilding4(:) yBuilding4(:) zBuilding4(:)];
% Update the obstacles with new probability values and display the map.
obs = 0.65;
updateOccupancy(map3D,xyzBuildings,obs)

figure
show(map3D)