function addPointsToMap(occMap, pose, PointsC, params)
    % Return if there are no points to be added
    if ~size(PointsC, 1)
        disp('addPointsToMap -- No visible points to be added to the voxel map');
        return;
    end
    % Camera pose with orientation part in quaternion form
    poseCamQuat = poseEul2Quat(getCamPose(pose, params));
    % Insertion in voxel map
    insertPointCloud(occMap, poseCamQuat, PointsC, params.camMaxRange);
end

