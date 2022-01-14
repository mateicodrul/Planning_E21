function setOccupancyBox(occMap, pose, params, occVal, scale_box)
    epsilon = 1e-3;
    disc_step_box = params.boundingBox ./ ceil((params.boundingBox + epsilon) * params.mapResolution);
    [disc_grid_box_X, disc_grid_box_Y, disc_grid_box_Z] = meshgrid(...
                -0.5*scale_box*params.boundingBox(1) : disc_step_box(1) : 0.5*scale_box*params.boundingBox(1), ...
                -0.5*scale_box*params.boundingBox(2) : disc_step_box(2) : 0.5*scale_box*params.boundingBox(2), ...
                -0.5*scale_box*params.boundingBox(3) : disc_step_box(3) : 0.5*scale_box*params.boundingBox(3));
    disc_points_box = [disc_grid_box_X(:) disc_grid_box_Y(:) disc_grid_box_Z(:)];
    disc_points_box = pose(1:3).' + disc_points_box;
    setOccupancy(occMap, disc_points_box, occVal);
end