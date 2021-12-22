function gain = computeGain(occMap, state, params)
    % Position state of node for which we are computing the gain
    origin = state(1:3).';
    % Find all the voxels within the allowed distance, gainRange, from
    % origin, which are also within the FOV
    % Voxels in world frame
    [voxels_X, voxels_Y, voxels_Z] = meshgrid(...
                max(origin(1) - params.gainRange, params.minXYZ(1)) : 1/params.mapResolution : min(origin(1) + params.gainRange, params.maxXYZ(1)), ...
                max(origin(2) - params.gainRange, params.minXYZ(2)) : 1/params.mapResolution : min(origin(2) + params.gainRange, params.maxXYZ(2)), ...
                max(origin(3) - params.gainRange, params.minXYZ(3)) : 1/params.mapResolution : min(origin(3) + params.gainRange, params.maxXYZ(3)) );
    voxels = [voxels_X(:) voxels_Y(:) voxels_Z(:)];
    % Get voxels in camera frame
    HC_W = extrinsicsFromPose(state);
    voxelsC = applyHomTransform(voxels, HC_W);
    % Keep only the voxels in gainRange and FOV
    [azim, elev, r] = cart2sph(voxelsC(:,1), voxelsC(:,2), voxelsC(:,3));
    in_range_and_fov_idx = inRangeAndFOV(azim, elev, r, params, 'gain'); 
    voxels = voxels(in_range_and_fov_idx, :);
    % These voxels are in range, but they are not necessarily visible from
    % origin, since they may be obstructed by occupied voxels
    
    % Find out which voxels are visible from origin. To get the directions 
    % in the camera frame, we should only rotate them, without applying any
    % translation
    directionsC = applyHomTransform((voxels - origin), HC_W);
    %directionsC = (HC_W(1:3, 1:3) * (voxels - origin).').';
    [intersections, ~] = rayIntersection(occMap, poseEul2Quat(state).', directionsC, params.camMaxRange);
    intersectionsC = applyHomTransform(intersections, HC_W);
    voxelsC = applyHomTransform(voxels, HC_W);
    visible_idx = vecnorm(intersectionsC, 2, 2) >= vecnorm(voxelsC, 2, 2);
    voxels = voxels(visible_idx, :);
    % Get occupancy values of visible voxels
    occupancyVals = checkOccupancy(occMap, voxels);
    % Gain computation
    gain = 0;
    gain = gain + params.giFree * sum(occupancyVals == 0);
    gain = gain + params.giOcc * sum(occupancyVals == 1);
    gain = gain + params.giUnmapped * sum(occupancyVals == -1);
    sum(occupancyVals == 0)
    sum(occupancyVals == 1)
    sum(occupancyVals == -1)
    % Scale gain with volume
    gain = gain * 1/params.mapResolution^3;
end
