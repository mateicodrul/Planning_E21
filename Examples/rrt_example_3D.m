clc
clear
close all

% Create a 3-D occupancy map and associated state validator. Plan, validate,
% and visualize a path through the occupancy map.

% Load a 3-D occupancy map of a city block.
% Specify a threshold for which cells to consider as obstacle-free.
mapData = load('dMapCityBlock.mat');
omap = mapData.omap;
omap.FreeThreshold = 0.5;
% Inflate the occupancy map to add a buffer zone for safe operation around
%the obstacles.
inflate(omap,1)

% Create an SE(3) state space object with bounds for state variables.
ss = stateSpaceSE3([-20 220;
    -20 220;
    -10 100;
    inf inf;
    inf inf;
    inf inf;
    inf inf]);

% Create a 3-D occupancy map state validator using the created state space.
sv = validatorOccupancyMap3D(ss);

% Assign the occupancy map to the state validator object. Specify the
% sampling distance interval.
sv.Map = omap;
sv.ValidationDistance = 0.1;

%% Plan and Visualize Path

% Create a path planner with increased maximum connection distance.
% Reduce the maximum number of iterations.
planner = plannerRRT(ss,sv);
planner.MaxConnectionDistance = 50;
planner.MaxIterations = 100;

% Create a user-defined evaluation function for determining whether the 
% path reaches the goal. Specify the probability of choosing the goal state
% during sampling.
planner.GoalReachedFcn = @(~,x,y)(norm(x(1:3)-y(1:3))<5);
planner.GoalBias = 0.1;
% Set the start and goal states.
start = [40 180 25 0.7 0.2 0 0.1];
goal = [150 33 35 0.3 0 0.1 0.6];

% Plan a path using the specified start, goal, and planner.
[pthObj,solnInfo] = plan(planner,start,goal);
% Check that the points of the path are valid states.
isValid = isStateValid(sv,pthObj.States);
if sum(isValid) == 7
    disp('States are valid')
else
    disp('Some states are invalid')
end
% Check that the motion between each sequential path state is valid.
isPathValid = zeros(size(pthObj.States,1)-1,1,'logical');
for i = 1:size(pthObj.States,1)-1
    [isPathValid(i),~] = isMotionValid(sv,pthObj.States(i,:),...
        pthObj.States(i+1,:));
end
if sum(isPathValid) == 6
    disp('State transistions are valid')
else
    disp('Some state transitions are invalid')
end
% Visualize the results
show(omap)
hold on
scatter3(start(1,1),start(1,2),start(1,3),'g','filled') % draw start state
scatter3(goal(1,1),goal(1,2),goal(1,3),'r','filled')    % draw goal state
plot3(pthObj.States(:,1),pthObj.States(:,2),pthObj.States(:,3),...
    'r-','LineWidth',2) % draw path
