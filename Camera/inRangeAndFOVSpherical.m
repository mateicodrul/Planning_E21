function in_range_and_fov_idx = inRangeAndFOVSpherical(voxelsC, params)
    [azim, elev, r] = cart2sph(voxelsC(:,3), -voxelsC(:,1), -voxelsC(:,2));
    azim_in_fov_idx = azim >= -params.fovHorizontal/2 & azim <= params.fovHorizontal/2;
    elev_in_fov_idx = elev >= -params.fovVertical/2 & elev <= params.fovVertical/2;
    max_range = params.gainRange;
    min_range = 0.5;
    r_in_range_idx = r > min_range & r <= max_range;
    in_range_and_fov_idx = azim_in_fov_idx & elev_in_fov_idx & r_in_range_idx;
end