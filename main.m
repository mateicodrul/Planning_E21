% Load files, functions and variables
setupSim;
tol_idx = 1;
tol_close = 0.4;

% Plot original model
%{
figure
set(gcf,'units','centimeters','position',[51,0,51,25.5])
subplot(1,3,1)
pcshow(Points, 'BackgroundColor', [1 1 1])
colormap turbo
view(-45,10)
%}

% Camera pose in world frame
pose = [0 -22/2 15/2 0 0 pi/2].';
%pose = [-35 0 0 0 0 0].';
% Extrinsics from pose
HC_W = extrinsicsFromPose(pose);
% Pose with orientation part in quaternion form
poseQuat = poseEul2Quat(pose);

% Get scan of visible environment from camera
PointsC_visible = depthCameraSim(pose, Points, params, tol_idx, tol_close);
% Add visible pointcloud in sensor coordinates to the occupancy map. Return
% if there are no points to be added
if ~size(PointsC_visible, 2)
    disp('No visible points to be added to the voxel map');
    close all;
    return;
end
insertPointCloud(occMap, poseQuat, PointsC_visible, params.camMaxRange + 1/params.mapResolution);

% Set all the voxels within the sensor range to free if no obstacles exist
% along the corresponding rays
Points_uf = findUnmappedFree(occMap, pose, params);
setOccupancy(occMap, Points_uf, 0);

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

showExploredMap(PointsC_visible, occMap, pose, pose)

%{
% Plot visible point cloud in world frame, on top of the full pointcloud
Points_visible = applyHomTransform(PointsC_visible, invH(HC_W));
hold on
pcshow(Points_visible, color(2,:), 'BackgroundColor', [1 1 1], 'MarkerSize', 50)
showCamera(pose)
hold off
set(gca,'FontSize',14,'TickLabelInterpreter','latex')

% Plot voxel map
subplot(1,3,2)
show(occMap)
hold on
showCamera(pose)
colormap turbo
title('Discovered Voxel Map $\hat{\mathcal{M}}$','Interpreter','Latex')
view(-45,10)
axis equal
grid on
hold off
set(gca,'FontSize',14,'TickLabelInterpreter','latex')
%set(gcf,'units','centimeters','position',[0,0,50,25])



%% See occupancy 

% Define 3D meshgrid of a cube with side length of 2*max_range and centered
% at the location of the camera in the world
[X,Y,Z] = meshgrid((pose(1) - params.camMaxRange):1/(params.mapResolution):(pose(1) + params.camMaxRange),...
                   (pose(2) - params.camMaxRange):1/(params.mapResolution):(pose(2) + params.camMaxRange),...
                   (pose(3) - params.camMaxRange):1/(params.mapResolution):(pose(3) + params.camMaxRange));
% Vectorize grid
X = X(:);
Y = Y(:);
Z = Z(:);
% Get occupancy at all grid points
Occ = getOccupancy(occMap, [X Y Z]);
% Distiguish betweem free and occupied
occ_bool_free = Occ < 0.5;
occ_bool_occ = Occ > 0.5;
% Free voxels are green and occupied voxels are red
occ_color_free = zeros(sum(occ_bool_free),3);
occ_color_occ = zeros(sum(occ_bool_occ),3);
occ_color_free(:,2) = 255;
occ_color_occ(:,1) = 255;

subplot(1,3,3)
scatter3(X(occ_bool_free), Y(occ_bool_free), Z(occ_bool_free), 60, occ_color_free,'s','filled')
hold on
scatter3(X(occ_bool_occ), Y(occ_bool_occ), Z(occ_bool_occ), 60, occ_color_occ,'s','filled')
showCamera(pose)
title('Discovered Occupancy Map $\hat{\mathcal{M}}_{\mbox{occ}}$','Interpreter','Latex')
axis equal
view(-45,10)
hold off
set(gca,'FontSize',14,'TickLabelInterpreter','latex')
%}