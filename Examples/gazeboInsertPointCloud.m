function PointsC = gazeboInsertPointCloud(subDepth, params)
    %%
    for i = 1:3
        depth_msg = receive(subDepth);
    end
    depth_image = double(rosReadImage(depth_msg));
    depth_image_nan = depth_image;
    depth_image(isnan(depth_image)) = params.camMaxDepth;

    Map = occupancyMap3D(10,...
                         'FreeThreshold', params.mapFreeThresh,...
                          'OccupiedThreshold', params.mapOccThresh);
    Map_free = occupancyMap3D(10,...
                              'FreeThreshold', params.mapFreeThresh,...
                              'OccupiedThreshold', params.mapOccThresh);
    Map_nan = occupancyMap3D(10,...
                             'FreeThreshold', params.mapFreeThresh,...
                             'OccupiedThreshold', params.mapOccThresh);
    Map_nan_free = occupancyMap3D(10,...
                                  'FreeThreshold', params.mapFreeThresh,...
                                  'OccupiedThreshold', params.mapOccThresh);

    % Get point cloud using depth
    invK = inv(params.K);
    PointsCFromDepth = zeros(size(depth_image, 1) * size(depth_image, 2), 3);
    PointsCFromDepth_nan = zeros(size(depth_image, 1) * size(depth_image, 2), 3);
    k = 1;
    for x = 1:size(depth_image, 2)
        for y = 1:size(depth_image, 1)
            P = (invK * depth_image(y, x) * [x; y; 1]).';
            P_nan = (invK * depth_image_nan(y, x) * [x; y; 1]).'; 
            PointsCFromDepth(k, :) = P;
            PointsCFromDepth_nan(k, :) = P_nan;
            k = k + 1;
        end
    end
    PointsC = PointsCFromDepth;
    PointsC_nan = PointsCFromDepth_nan;
    Points = applyHomTransform(PointsC, HFromPose(pose) * params.HB_C);
    Points_nan = applyHomTransform(PointsC_nan, HFromPose(pose) * params.HB_C);
    PointsC_nan(any(isnan(PointsC_nan), 2), :) = [];
    addPointsToMap(Map, pose, PointsC, params);
    addPointsToMap(Map_nan, pose, PointsC_nan, params);

    occ_check_divider = 2;
    [X,Y,Z] = meshgrid(params.minXYZ(1):1/(occ_check_divider * params.mapResolution):params.maxXYZ(1),...
                       params.minXYZ(2):1/(occ_check_divider * params.mapResolution):params.maxXYZ(2),...
                       params.minXYZ(3):1/(occ_check_divider * params.mapResolution):params.maxXYZ(3));
    % Vectorize grid
    X = X(:);
    Y = Y(:);
    Z = Z(:);
    % Get occupancy at all grid points
    isOccupied = checkOccupancy(Map, [X Y Z]);
    isOccupied_nan = checkOccupancy(Map_nan, [X Y Z]);
    % Distiguish betweem free and occupied
    occ_bool_free = isOccupied == 0;
    occ_bool_occ = isOccupied == 1;
    occ_bool_free_nan = isOccupied_nan == 0;
    occ_bool_occ_nan = isOccupied_nan == 1;
    
    Points_free = [X(occ_bool_free), Y(occ_bool_free), Z(occ_bool_free)];
    PointsC_free = applyHomTransform(Points_free, invH(HFromPose(pose) * params.HB_C));
    addPointsToMap(Map_free, pose, PointsC_free, params);
    
    Points_free_nan = [X(occ_bool_free_nan), Y(occ_bool_free_nan), Z(occ_bool_free_nan)];
    PointsC_free_nan = applyHomTransform(Points_free_nan, invH(HFromPose(pose) * params.HB_C));
    addPointsToMap(Map_nan_free, pose, PointsC_free_nan, params);

    figure
    set(gcf,'units','centimeters','position',[0,0,20,24])
    ax(1) = subplot(3,2,1);
    pcshow(Points_nan,'BackgroundColor', [1 1 1], 'MarkerSize', 10)
    view(-45,45)
    axis equal
    axis off
    ax(2) = subplot(3,2,3);
    show(Map_nan)
    title('')
    view(-45,45)
    axis equal
    axis off
    ax(3) = subplot(3,2,5);
    show(Map_nan_free)
    title('')
    view(-45,45)
    axis equal
    axis off

    ax(4) = subplot(3,2,2);
    pcshow(Points,'BackgroundColor', [1 1 1], 'MarkerSize', 10)
    view(-45,45)
    axis equal
    axis off
    ax(5) = subplot(3,2,4);
    show(Map)
    title('')
    view(-45,45)
    axis equal
    axis off
    ax(6) = subplot(3,2,6);
    show(Map_free)
    title('')
    view(-45,45)
    axis equal
    axis off

    colormap(ax(1),turbo)
    colormap(ax(2),turbo)
    colormap(ax(3),summer)

    colormap(ax(4),turbo)
    colormap(ax(5),turbo)
    colormap(ax(6),summer)

    saveas(gcf,'OctoMap_integrate','svg')
end

