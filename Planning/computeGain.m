function gain = computeGain(occMap, state, params)
    % Initilaize gain to 0
    gain = 0;    
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
    % If there are no voxels in range, gain is 0
    if ~size(voxels,1)
        return;
    end
    % These voxels are in range, but they are not necessarily visible from
    % origin, since they may be obstructed by occupied voxels
    %sum(checkOccupancy(occMap, voxels) == 1) 
    
    % Find out which voxels are visible from origin. To get the directions 
    % in the camera frame, we should only rotate them, without applying any
    % translation
    directionsC = (HC_W(1:3, 1:3) * (voxels - origin).').';
    intersections = zeros(size(directionsC));
    isOccupied = zeros(size(directionsC, 1), 1);
    for dirCIdx = 1:size(directionsC, 1)
        dirC = directionsC(dirCIdx,:);
        max_range = norm(dirC, 2) + 2;
        [inter, isOcc] = rayIntersection(occMap, poseEul2Quat(state).', dirC, max_range);
        intersections(dirCIdx,:) = inter;
        isOccupied(dirCIdx,:) = isOcc;
    end
    % Keep unique intersections
    [~, intersectionsIdx] = unique(intersections,'rows','stable');
    isOccupied = isOccupied(intersectionsIdx);
    % If there are no visible voxels, gain is 0
    if isempty(isOccupied)
        return;
    end
    % Gain computation
    gain = gain + params.giFree * sum(isOccupied == 0);
    gain = gain + params.giOcc * sum(isOccupied == 1);
    gain = gain + params.giUnmapped * sum(isOccupied == -1);

    %{
    intersectionsC = applyHomTransform(intersections, HC_W);
    voxelsC = applyHomTransform(voxels, HC_W);
    visible_idx = vecnorm(intersectionsC, 2, 2) >= vecnorm(voxelsC, 2, 2);
    voxels = voxels(visible_idx, :);
    % If there are no visible voxels, gain is 0
    if ~size(voxels,1)
        return;
    end
    % If there are visible voxels, get their occupancy values and use them
    % to compute the gain
    occupancyVals = checkOccupancy(occMap, voxels);
    % Gain computation
    gain = gain + params.giFree * sum(occupancyVals == 0);
    gain = gain + params.giOcc * sum(occupancyVals == 1);
    gain = gain + params.giUnmapped * sum(occupancyVals == -1);
    %}

    % Scale gain with volume
    gain = gain * 1/params.mapResolution^3;
end