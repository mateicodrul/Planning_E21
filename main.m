% Set up simulation
setupSim;

% Start timer
tic

% Publish intitial pose reference message and wait for the drone to move
% Ensure that the camera is looking toward the inner part of the box where 
% the object lies
% Decode pose messages to get initial pose of the drone in the world frame
pose = gazeboGetPoseInit(subPose);
posePrev = pose;
poseRef = [0.5 0.5 1 0 0 pi/4].';
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
setOccupancyBox(occMap, pose, params, 0, 2)

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

%% NBV planner
NBV

% Shutdown ROS node
rosshutdown

%% Save results
save(['Logging/', logfile_name],'occMap','mapping_progress_vect','time_vect','pose_vect','params')

