function PointsC_boundary = generateVirtualBoundary(params)
    azim_step = 0.5 / (params.mapResolution) / params.camMaxRange;
    elev_step = 0.5 / (params.mapResolution) / params.camMaxRange;
    azim = -params.fovHorizontal/2 : azim_step : params.fovHorizontal/2;
    elev = -params.fovVertical/2 : elev_step : params.fovVertical/2;
    [az, el] = meshgrid(azim, elev);
    [x, y, z] = sph2cart(az(:), el(:), params.camMaxRange*ones(length(az(:)),1)); 
    PointsC_boundary = [x y z];
    % PointsC_boundary = uniquetol([x y z], 0.025, 'ByRows', true);
end