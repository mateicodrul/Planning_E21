function PointsB = gazeboGetCameraPoints(subPointcloud, params)
    pointcloud_msg = subPointcloud.LatestMessage;
    PointsC = readXYZ(pointcloud_msg);
    PointsC(any(isnan(PointsC), 2), :) = [];
    PointsB = applyHomTransform(PointsC, params.HB_C);
end