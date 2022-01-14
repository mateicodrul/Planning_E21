function PointsC = gazeboGetCameraPoints(subPointcloud)
    % Obtain latest image message from the camera
    %pointcloud_msg = subPointcloud.LatestMessage;
    for i = 1:3
        pointcloud_msg = receive(subPointcloud);
    end
    PointsC = rosReadXYZ(pointcloud_msg);
    % Remove NaNs
    PointsC(any(isnan(PointsC), 2), :) = [];
end