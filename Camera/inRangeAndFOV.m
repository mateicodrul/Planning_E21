function in_range_and_fov_idx = inRangeAndFOV(azim, elev, r, params, range_select)
    azim_in_fov_idx = azim >= -params.fovHorizontal/2 & azim <= params.fovHorizontal/2;
    elev_in_fov_idx = elev >= -params.fovVertical/2 & elev <= params.fovVertical/2;
    if strcmp(range_select, 'gain')
        max_range = params.gainRange;
        min_range = 10e-3;
    elseif strcmp(range_select, 'camera')
        max_range = params.camMaxRange;
        min_range = 10e-3;
    else
        disp('inRangeAndFOV -- unspecified range case. Choose between camera and gain');
    end
    r_in_range_idx = r > min_range & r <= max_range;
    in_range_and_fov_idx = azim_in_fov_idx & elev_in_fov_idx & r_in_range_idx;
end