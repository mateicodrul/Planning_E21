function collision_free_flag = checkCollisionAlongPath_v3(occMap, startState, direction, params)
    % Include some overshoot into the direction to to stay safe
    direction = direction + direction ./ norm(direction) * params.collisionOvershoot;
    % Discretize the box to check for collisions around each discrete
    % pose along the path. The discretization step (in meters) along each 
    % axis should be smaller than 1/params.mapResolution
    epsilon = 1e-3;
    disc_step_box = params.boundingBox ./ ...
                    ceil((params.boundingBox + epsilon) * params.mapResolution);
    [disc_grid_box_X, disc_grid_box_Y, disc_grid_box_Z] = meshgrid(...
                -params.boundingBox(1)/2 : disc_step_box(1) : params.boundingBox(1)/2, ...
                -params.boundingBox(2)/2 : disc_step_box(2) : params.boundingBox(2)/2, ...
                -params.boundingBox(3)/2 : disc_step_box(3) : params.boundingBox(3)/2);
    disc_points_box = [disc_grid_box_X(:) disc_grid_box_Y(:) disc_grid_box_Z(:)];
    % Discretize direction vector to obtain discrete points to center the
    % box at
    disc_step_dir = min(disc_step_box);
    num_disc_step_dir = ceil(norm(direction) / disc_step_dir);
    discState = startState;
    for i = 1:num_disc_step_dir
        discState = discState + direction * disc_step_dir/norm(direction);
        % Get occupancy in a box around ech discrete points along the
        % direction. As soon as we find something not free, we return
        occupancy = getOccupancy(occMap, [discState.' + disc_points_box]);
        if sum(occupancy < params.mapFreeThresh) > 0
           collision_free_flag = false;
           return; 
        end
    end
    % If this point is reaached, the path is free
    collision_free_flag = true;
end