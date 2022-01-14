setupSim
load('Logging/duck1.mat')

pose = pose_vect(end,:).';
showExploredMap(occMap, pose, pose, [], params);
showOccupancy(occMap, pose, params, 1);

%%
poseRef = pose;
gazeboMoveDroneSampled(pubTrajectory, pose, poseRef, params);
% Decode pose messages to get pose of the drone in the world frame
pose = gazeboGetPose(subPose, poseRef);
pose_vect = [pose_vect; pose.'];

% Get scan of visible environment from Gazebo camera
PointsC = gazeboGetCameraPointsFromDepth(subDepth, params, 'depth');

% Add visible pointcloud in camera coordinates to the occupancy map 
addPointsToMap(occMap, pose, PointsC, params);

% Optionally, set zero occupancy within box around start pose. This box has 
% a size that is a multiple of the size of the collsion box
setOccupancyBox(occMap, pose, params, 0, 4)

% Plot explored map
if params.showMap
    showExploredMap(occMap, pose, pose, PointsC, params);
end
%showOccupancy(occMap, pose, params, 1)

% Progress
mapping_progress_vect = [mapping_progress_vect; mappingProgress(occMap, voxels)];
elapsed_time = toc;
time_vect = [time_vect; elapsed_time]; 
% Plot progress
if params.showProgress
    showMapProgress(time_vect, mapping_progress_vect);
end

NBV