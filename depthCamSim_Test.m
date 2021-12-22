clc
clear
close all

% Load model of real environment
stl_file_name = '3DBenchy';
load(['./Source_Models/' stl_file_name '.mat'])
%load(['./Source_Models/' stl_file_name '_high_density.mat'])

% Simple rotations in 3D
Rx = @(theta) [1 0 0;
               0 cos(theta) -sin(theta);
               0 sin(theta) cos(theta)];
Ry = @(theta) [cos(theta) 0 sin(theta);
               0 1 0;
               -sin(theta) 0 cos(theta)];
Rz = @(theta) [cos(theta) -sin(theta) 0;
               sin(theta) cos(theta) 0;
               0 0 1];
           
% Intrinsic RPY rotation in 3D 
Rzyx = @(yaw, pitch, roll) Rz(yaw) * Ry(pitch) * Rx(roll);         
           
% Homogeneous transformation in 3D
H = @(R,t) [R t;
            zeros(1,3) 1];

% Camera parameters
f = 500;
img_width = 1920;
img_height = 1080;
max_range = 10;
fov_W = 2*atan(img_width/(2*f));
fov_H = 2*atan(img_height/(2*f));
% Camera pose in world coordinates. pose(1:3) is the position of camera
% center wrt the world origin, expressed on world coordinates
pose = [-35; 0; 0; 0; 0; 0];
RW_C = Rzyx(pose(6), pose(5), pose(4));
tW_WC = pose(1:3);
RC_W = RW_C.';
tC_CW = -RC_W * tW_WC;
HC_W = H(RC_W, tC_CW);

% Point cloud in camera frame
Points_t = cart2hom(Points);
PointsC_t = (HC_W * Points_t.').';
PointsC = hom2cart(PointsC_t);
% Point cloud in range of camera
dist_to_points = vecnorm(PointsC.', 2).';
inrange_points_bool = dist_to_points < max_range;
PointsC_inrange = PointsC(inrange_points_bool,:);

% Plot full point cloud and the part in range, both in camera coordinates 
figure
pcshow(PointsC)
figure
pcshow(PointsC_inrange)

% Part of pointcloud in range that the camera can actually see (avoid
% seeing through objects)
[azim, elev, r] = cart2sph(PointsC_inrange(:,1), PointsC_inrange(:,2), PointsC_inrange(:,3));
[elev, elev_sort_idx] = sort(elev);
azim = azim(elev_sort_idx);
r = r(elev_sort_idx);

% Tolerance that changes based on distance to closest point in pointcloud
angle_tol_base = 1e-1;
angle_tol_inv = 5 * 1 / (10*min(r));
angle_tol_exp = 20 * exp(-min(r));
angle_tol_vec = [angle_tol_base angle_tol_inv angle_tol_exp].';

for k = 1:3

angle_tol = angle_tol_vec(k);
[~, elev_sort_unique_idx, ~] = uniquetol(elev, angle_tol);
elev_sort_unique_idx = sort(elev_sort_unique_idx);
elev_sort_unique_idx = [0; elev_sort_unique_idx];
numel(elev_sort_unique_idx)

PointsC_visible = [];
for i = 2:length(elev_sort_unique_idx)
    azim_at_this_elev = azim( (elev_sort_unique_idx(i-1) + 1):elev_sort_unique_idx(i) );
    r_at_this_elev = r( (elev_sort_unique_idx(i-1) + 1):elev_sort_unique_idx(i) );
    
    [azim_at_this_elev, azim_at_this_elev_sort_idx] = sort(azim_at_this_elev);
    r_at_this_elev = r_at_this_elev(azim_at_this_elev_sort_idx);
    
    [~, azim_at_this_elev_sort_unique_idx, ~] = uniquetol(azim_at_this_elev, angle_tol);
    azim_at_this_elev_sort_unique_idx = sort(azim_at_this_elev_sort_unique_idx);
    azim_at_this_elev_sort_unique_idx = [0; azim_at_this_elev_sort_unique_idx];
    
    for j = 2:(length(azim_at_this_elev_sort_unique_idx) - 1)
        r_at_this_elev_and_azim = r_at_this_elev( (azim_at_this_elev_sort_unique_idx(j-1) + 1):azim_at_this_elev_sort_unique_idx(j) );
        r_visible = min(r_at_this_elev_and_azim);
        [x, y, z] = sph2cart(azim_at_this_elev( azim_at_this_elev_sort_unique_idx(j) ),...
                             elev( elev_sort_unique_idx(i) ), ...
                             r_visible);
        PointsC_visible = [PointsC_visible; x y z];
    end
end

figure
pcshow(PointsC_visible)
title(['Tolerance ' num2str(k)])
% scatter3(PoinctsC_inrange(:,1),PoinctsC_inrange(:,2),PoinctsC_inrange(:,3),'.r')
end

%%
% k = boundary(PoinctsC_valid);
% figure
% pcshow(PoinctsC_valid(k(:,2),:),'b');