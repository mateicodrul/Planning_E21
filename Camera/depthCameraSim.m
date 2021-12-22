%{
Description
-----------
Get a pointcloud of the visible environment using a simulated depth camera
% !!! Assume for now that the camera frame has the x-axis pointing outwards
% and aligned with the principal axis

Input(s):
--------
@ pose = Camera pose in world coordinates. pose(1:3) is the position of 
camera center wrt the world origin, expressed in world coordinates
@ Points = Full pointcloud of the real environment, expressed in world 
coordinates
@ params = structure with defined parameters
@ tol_idx = type of tolerance to use: 1 = constant, 2 = inverse,
3 = exponential
@ tol_close = distance tolerance used for pointcloud augumentation

Output(s):
--------
@ PointsC_visible = Portion of pointcloud that the camera can observe from
its current pose, expressed in camera coordinates
%}

function PointsC_visible = depthCameraSim(pose, Points, params, tol_idx, tol_close)
    % Get extrinsiscs from pose
    HC_W = extrinsicsFromPose(pose);
    % Point cloud in camera frame   
    PointsC = applyHomTransform(Points, HC_W);

    % Eliminate points outside the FOV and outside the range (may not be 
    % necessary with a real camera)
    [azim, elev, r] = cart2sph(PointsC(:,1), PointsC(:,2), PointsC(:,3));
    in_range_and_fov_idx = inRangeAndFOV(azim, elev, r, params, 'camera'); 
    if ~sum(in_range_and_fov_idx)
        PointsC_visible = [];
        disp('depthCameraSim -- No points in range and FOV');
        return;
    end
    PointsC_inrange = PointsC(in_range_and_fov_idx, :);
    azim = azim(in_range_and_fov_idx);
    elev = elev(in_range_and_fov_idx);
    r = r(in_range_and_fov_idx);

    % Plot point cloud in range, in camera coordinates 
    %figure
    %pcshow(PointsC_inrange)
    
    % Find part of pointcloud in range that the camera can actually see
    % (avoid seeing through objects; not a problem in the real world. Here
    % it happens due to use of a pointcloud as a representation of the real
    % object))
    [elev, elev_sort_idx] = sort(elev);
    azim = azim(elev_sort_idx);
    r = r(elev_sort_idx);

    % Tolerance that changes based on distance to closest point in pointcloud
    angle_tol_base = 1e-1;
    angle_tol_inv = 5 * 1 / (10*min(r));
    angle_tol_exp = 20 * exp(-min(r));
    angle_tol_vec = [angle_tol_base angle_tol_inv angle_tol_exp].';
    angle_tol = angle_tol_vec(tol_idx);
    
    [~, elev_sort_unique_idx, ~] = uniquetol(elev, angle_tol);
    elev_sort_unique_idx = sort(elev_sort_unique_idx);
    elev_sort_unique_idx = [0; elev_sort_unique_idx];
    %numel(elev_sort_unique_idx)

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

    % PointsC_visible obtained before may be too sparse. We add points
    % from PointsC_inrange that are close to the ones in PointsC_visible
    % to increase density
    if tol_close > 0
        num_visible = size(PointsC_visible,1);
        for i = 1:num_visible
            point_difference = PointsC_visible(i,:) - PointsC_inrange;
            close_points_bool = vecnorm(point_difference.', 2).' < tol_close;
            PointsC_visible = [PointsC_visible; PointsC_inrange(close_points_bool,:)];
        end
        PointsC_visible = unique(PointsC_visible,'rows');
    end
end