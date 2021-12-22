% Load files, functions and variables
setupSim;
tol_idx = 1;
tol_close = 0.4;

params.mapResolution = 4;
occMap = occupancyMap3D(params.mapResolution,...
                        'FreeThreshold', params.mapFreeThresh,...
                        'OccupiedThreshold', params.mapOccThresh);

% Camera pose in world frame
pose = [0 -22/2 15/2 0 0 pi/2].';
% Extrinsics from pose
HC_W = extrinsicsFromPose(pose);
% Pose with orientation part in quaternion form
poseQuat = poseEul2Quat(pose);

% Set all the voxels within the sensor range to free if no obstacles exist
% along the corresponding rays
Points_uf = findUnmappedFree(occMap, pose, params);
%insertPointCloud(occMap, poseQuat, applyHomTransform(Points_uf, HC_W), params.camMaxRange + 1/params.mapResolution);
tic
setOccupancy(occMap, Points_uf, 0);
toc

PointsC_boundary = generateVirtualBoundary(params);
%insertPointCloud(occMap, poseQuat, PointsC_boundary, params.camMaxRange + 1/params.mapResolution);

% Plot voxel map
figure
set(gcf,'units','centimeters','position',[51,0,50,25])
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