function gain = computeGainDebug(occMap, state, params)
    %%
    state = [5 1 1 0 0 0].';
    % Initilaize gain to 0
    gain = 0;
    % Get the camera state corresponding to the node for whcih we are
    % computing the gain
    stateCam = getCamPose(state, params);
    % Position state
    origin = stateCam(1:3).';
    % Find all the voxels within the allowed distance, gainRange, from
    % origin, which are also within the FOV
    % Voxels in world frame
    [voxels_X, voxels_Y, voxels_Z] = meshgrid(...
                max(origin(1) - params.gainRange, params.minXYZ(1)) : 1/params.mapResolution : min(origin(1) + params.gainRange, params.maxXYZ(1)), ...
                max(origin(2) - params.gainRange, params.minXYZ(2)) : 1/params.mapResolution : min(origin(2) + params.gainRange, params.maxXYZ(2)), ...
                max(origin(3) - params.gainRange, params.minXYZ(3)) : 1/params.mapResolution : min(origin(3) + params.gainRange, params.maxXYZ(3)));
    voxels = [voxels_X(:) voxels_Y(:) voxels_Z(:)] + 0.5/params.mapResolution;
    % Get voxels in camera frame
    HC_W = invH(HFromPose(state) * params.HB_C);
    voxelsC = applyHomTransform(voxels, HC_W);
    % Keep only the voxels in gainRange and FOV
    %in_range_and_fov_idx_spherical = inRangeAndFOVSpherical(voxelsC, params); 
    in_range_and_fov_idx = inRangeAndFOV(voxelsC, params); 
    voxels = voxels(in_range_and_fov_idx, :);
    % If there are no voxels in range, gain is 0
    if ~size(voxels,1)
        return;
    end
    % These voxels are in range, but they are not necessarily visible from
    % origin, since they may be obstructed by occupied voxels

    % Find out which voxels are visible from origin. To get the directions 
    % in the camera frame, we should only rotate them, without applying any
    % translation
    directionsC = (HC_W(1:3, 1:3) * (voxels - origin).').';
    % Set occupancy of all voxels in range to a number, not in the set 
    % {-1,0,1}
    intersections = zeros(size(directionsC));
    isOccupied = zeros(size(directionsC, 1), 1) + 2;
    for dirCIdx = 1:size(directionsC, 1)
        dirC = directionsC(dirCIdx,:);
        max_range = norm(dirC, 2);
        [inter, isOcc] = rayIntersection(occMap, poseEul2Quat(stateCam).', dirC, max_range);
        % If the distance between intersection and the actual voxel in
        % range is sufficiently small, it means the voxel is visible
        if norm(inter - voxels(dirCIdx,:), 2) <= 1e-4
            intersections(dirCIdx,:) = inter;
            isOccupied(dirCIdx,:) = isOcc;
        end
    end
    % Remove voxels for which occupancy is still 2 after calling rayIntersect
    voxels(isOccupied == 2,:) = [];
    intersections(~any(intersections, 2), : ) = [];
    % In practice, we only care about the occupancy values
    isOccupied(isOccupied == 2) = [];
    % Keep unique intersections
    % [~, intersectionsIdx] = unique(intersections,'rows','stable');
    % isOccupied = isOccupied(intersectionsIdx);
    % If there are no visible voxels, gain is 0
    if isempty(isOccupied)
        return;
    end

    % Gain computation
    gain = gain + params.giFree * sum(isOccupied == 0);
    gain = gain + params.giOcc * sum(isOccupied == 1);
    gain = gain + params.giUnmapped * sum(isOccupied == -1);

    % Scale gain with volume
    gain = gain * (1/params.mapResolution)^3;

    
    figure(1)
    scatter3(voxels(:,1), voxels(:,2), voxels(:,3))
    hold on
    %scatter3(intersections(:,1), intersections(:, 2), intersections(:, 3), 'filled')
    %scatter3(voxels(isOccupied == 0,1), voxels(isOccupied == 0,2), voxels(isOccupied == 0,3), 'filled')
    %hold off
    
end