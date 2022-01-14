function PointsC = gazeboGetCameraPointsFromDepth(subDepth, params, pointSource)
    % Obtain latest image message from the camera
    %depth_msg = subDepth.LatestMessage;
    for i = 1:2
        depth_msg = receive(subDepth);
    end
    depth_image = double(rosReadImage(depth_msg));
    
    % Add artificial points with depth params.camMaxDepth 
    depth_image(isnan(depth_image)) = params.camMaxDepth;

    % Get point cloud using depth
    if strcmp(pointSource, 'depth')
        invK = inv(params.K);
        PointsCFromDepth = zeros(size(depth_image, 1) * size(depth_image, 2), 3);
        k = 1;
        for x = 1:size(depth_image, 2)
            for y = 1:size(depth_image, 1)
                P = (invK * depth_image(y, x) * [x; y; 1]).';
                PointsCFromDepth(k, :) = P;
                k = k + 1;
            end
        end
        PointsC = PointsCFromDepth;
    end

    %{
    figure
    pcshow(PointsC)
    figure
    imagesc(depth_image)
    colorbar
    %}

    % Get point cloud by computing disparity first. Generally, a disparity
    % image will be obtained rather than a depth image, in which case the
    % first line after the if statement is not necessary
    if strcmp(pointSource, 'disparity')
        disparity_image = params.focalLength * params.Tx ./ depth_image;
        Q = [1 0 0 -params.K(1,3);
             0 1 0 -params.K(2,3);
             0 0 0 params.focalLength;
             0 0 1/params.Tx 0];
        PointsCFromDisparity = zeros(size(disparity_image, 1) * size(disparity_image, 2), 3);
        k = 1;
        for x = 1:size(disparity_image, 2)
            for y = 1:size(disparity_image, 1)
                P_t = Q * [x; y; disparity_image(y,x); 1];
                P_t = P_t ./ P_t(end);
                PointsCFromDisparity(k, :) = P_t(1:end-1).';
                k = k + 1;
            end
        end
        PointsC = PointsCFromDisparity;
    end
end

