function Points_uf = findUnmappedFree(occMap, pose, params)
    HC_W = extrinsicsFromPose(pose);    
    PointsC_boundary = generateVirtualBoundary(params);
    directionsC = PointsC_boundary ./ vecnorm(PointsC_boundary,2,2);
    [intersections, ~] = rayIntersection(occMap, poseEul2Quat(pose).', directionsC, params.camMaxRange + 1/params.mapResolution);
    intersectionsC = applyHomTransform(intersections, HC_W);
    free_idx = vecnorm(intersectionsC, 2, 2) >= params.camMaxRange;
    if ~sum(free_idx)
        Points_uf = [];
        return;
    end
    PointsC_uf = PointsC_boundary(free_idx,:);
    fraction_vect = 1/params.mapResolution/params.camMaxRange:1/params.mapResolution/params.camMaxRange:1;
    m = repmat(fraction_vect, size(PointsC_uf,1), 1);
    m = m(:);
    PointsC_uf = m .* repmat(PointsC_uf, length(fraction_vect), 1);
    Points_uf = applyHomTransform(PointsC_uf, invH(HC_W));
    Points_uf = uniquetol(Points_uf, 0.006, 'ByRows', true);
end