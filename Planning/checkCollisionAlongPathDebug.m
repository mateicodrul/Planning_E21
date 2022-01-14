function collision_free_flag = checkCollisionAlongPathDebug(occMap, startState, direction, params)
    % Inflate obstacles in the map
    inflate(copy(occMap), params.occInflationRadius);
    % Include some overshoot into the direction to stay safe
    direction = direction + direction ./ norm(direction) * params.collisionOvershoot;
    endState = startState + direction;
    % Discretize the box to check for collisions between endpoints.
    % The discretization step (in meters) along each axis should be smaller
    % than 1/params.mapResolution
    epsilon = 1e-3;
    disc_step_box = params.boundingBox ./ ...
                    ceil((params.boundingBox + epsilon) * params.mapResolution);
    [disc_grid_box_X, disc_grid_box_Y, disc_grid_box_Z] = meshgrid(...
                -params.boundingBox(1)/2 : disc_step_box(1) : params.boundingBox(1)/2, ...
                -params.boundingBox(2)/2 : disc_step_box(2) : params.boundingBox(2)/2, ...
                -params.boundingBox(3)/2 : disc_step_box(3) : params.boundingBox(3)/2);
    disc_points_box = [disc_grid_box_X(:) disc_grid_box_Y(:) disc_grid_box_Z(:)];
    % Check that no line that unites two corresponding points in the boxes
    % around start and end positions, respectively, intersects any obstacle
    % or unmapped voxel
    %showOccupancy(occMap, [startState; 0; 0; 0], params, 1);
    %hold on
    %showCollisionBox([startState; 0; 0; 0], [endState; 0; 0; 0], params);
    %hold on

    for i = 1:size(disc_points_box, 1)
        offset = disc_points_box(i,:);
        % No need to convert directions to camera frame, since we here
        % consider the coordinate frame associated with startState as being
        % parallel to the global coordinate frame, and translation should
        % not play a role, since direction is a difference between 2
        % vectors in world coordinates
        [intersection, ~] = rayIntersection(occMap, [startState.' + offset, 1, zeros(1,3)], direction.', params.camMaxRange, false);
        % If the intersection is closer than the endpoint difference,
        % it means a collision has been found along current direction, and
        % we return
        %so = startState.' + offset;
        %eo = endState.' + offset;
        %plot3([so(1) eo(1)], [so(2) eo(2)], [so(3) eo(3)], 'b')
        %plot3([so(1) intersection(1)], [so(2) intersection(2)], [so(3) intersection(3)], 'r')
        if norm(intersection - (startState.' + offset), 2) < norm(endState.' - startState.', 2)
           %disp('checkCollisionAlongPath -- Collision detected within box for the given direction')
           collision_free_flag = false;
           return; 
        end
    end
    % If this point is reached, the path is free
    collision_free_flag = true;
end
