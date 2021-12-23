% Load files, functions and variables
setupSim;

% Publish pose reference message and wait for the drone to move 
send(pubTrajectory, pose_ref_msg)
pause(5);
% Decode pose messages to get initial pose of the drone in the world frame
pose_msg = subPose.LatestMessage;
poseQuat = [pose_msg.Position.X, pose_msg.Position.Y, pose_msg.Position.Z,...
             pose_msg.Orientation.W, pose_msg.Orientation.X, pose_msg.Orientation.Y, pose_msg.Orientation.Z].';
pose = poseQuat2Eul(poseQuat);
% Pose with orientation part in quaternion form
poseQuat = poseEul2Quat(pose);

% Get scan in base frame of visible environment from Gazebo camera
PointsB = gazeboGetCameraPoints(subPointcloud, params);

% Add visible pointcloud in body coordinates to the occupancy map. Return
% if there are no points to be added
if ~size(PointsB, 2)
    disp('No visible points to be added to the voxel map');
    close all;
    return;
end
insertPointCloud(occMap, poseQuat, PointsB, params.camMaxRange + 1/params.mapResolution);

% Set all the voxels within the sensor range to free if no obstacles exist
% along the corresponding rays
% Points_uf = findUnmappedFree(occMap, pose, params);
% setOccupancy(occMap, Points_uf, 0);

% Set zero occupancy within box around start pose
epsilon = 1e-3;
disc_step_box = params.boundingBox ./ ceil((params.boundingBox + epsilon) * params.mapResolution);
[disc_grid_box_X, disc_grid_box_Y, disc_grid_box_Z] = meshgrid(...
            -params.boundingBox(1)/2 : disc_step_box(1) : params.boundingBox(1)/2, ...
            -params.boundingBox(2)/2 : disc_step_box(2) : params.boundingBox(2)/2, ...
            -params.boundingBox(3)/2 : disc_step_box(3) : params.boundingBox(3)/2);
disc_points_box = [disc_grid_box_X(:) disc_grid_box_Y(:) disc_grid_box_Z(:)];
disc_points_box = pose(1:3).' + disc_points_box;
setOccupancy(occMap, disc_points_box, 0);

%showExploredMap(PointsB, occMap, pose, pose)
