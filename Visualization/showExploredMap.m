function showExploredMap(PointsB, occMap, pose, posePrev)
    color = colororder;
    % Extrinsics from pose
    HB_W = extrinsicsFromPose(pose);

    figure(1)
    set(gcf,'units','centimeters','position',[51,0,51,25.5])

    %%% Plot voxel map %%%
    %subplot(1,3,2)
    show(occMap)
    hold on
    showCamera(pose)
    plot3([posePrev(1) pose(1)], [posePrev(2) pose(2)], [posePrev(3) pose(3)],...
          'LineStyle', '-', 'LineWidth', 1.5, 'Color', color(1,:),...
          'Marker', 'o', 'MarkerSize', 1,...
          'MarkerFaceColor', color(1,:), 'MarkerEdgeColor', color(1,:))
    colormap turbo
    title('Discovered Voxel Map $\hat{\mathcal{M}}$','Interpreter','Latex')
    view(-45,10)
    axis equal
    grid on
    set(gca,'FontSize',14,'TickLabelInterpreter','latex')
    
    %{
    %%% Plot visible point cloud in world frame %%%
    subplot(1,3,1)
    if ~isempty(PointsB)
        Points = applyHomTransform(PointsB, invH(HB_W));
        pcshow(Points, color(2,:), 'BackgroundColor', [1 1 1], 'MarkerSize', 50)
    end
    hold on
    showCamera(pose)
    plot3([posePrev(1) pose(1)], [posePrev(2) pose(2)], [posePrev(3) pose(3)],...
          'LineStyle', '-', 'LineWidth', 1.5, 'Color', color(1,:),...
          'Marker', 'o', 'MarkerSize', 1,...
          'MarkerFaceColor', color(1,:), 'MarkerEdgeColor', color(1,:))
    title('Point Cloud','Interpreter','Latex')
    view(-45,10)
    axis equal
    grid on
    set(gca,'FontSize',14,'TickLabelInterpreter','latex')
    %}

    %{
    %%% Plot occupancy values %%%
    % Define 3D meshgrid of a cube with side length of 2*max_range and centered
    % at the location of the camera in the world
    [X,Y,Z] = meshgrid((pose(1) - params.camMaxRange):1/(params.mapResolution):(pose(1) + params.camMaxRange),...
                       (pose(2) - params.camMaxRange):1/(params.mapResolution):(pose(2) + params.camMaxRange),...
                       (pose(3) - params.camMaxRange):1/(params.mapResolution):(pose(3) + params.camMaxRange));
    % Vectorize grid
    X = X(:);
    Y = Y(:);
    Z = Z(:);
    % Get occupancy at all grid points
    Occ = getOccupancy(occMap, [X Y Z]);
    % Distiguish betweem free and occupied
    occ_bool_free = Occ < 0.5;
    occ_bool_occ = Occ > 0.5;
    % Free voxels are green and occupied voxels are red
    occ_color_free = zeros(sum(occ_bool_free),3);
    occ_color_occ = zeros(sum(occ_bool_occ),3);
    occ_color_free(:,2) = 255;
    occ_color_occ(:,1) = 255;
    
    subplot(1,3,3)
    scatter3(X(occ_bool_free), Y(occ_bool_free), Z(occ_bool_free), 60, occ_color_free,'s','filled')
    hold on
    scatter3(X(occ_bool_occ), Y(occ_bool_occ), Z(occ_bool_occ), 60, occ_color_occ,'s','filled')
    showCamera(pose)
    title('Discovered Occupancy Map $\hat{\mathcal{M}}_{\mbox{occ}}$','Interpreter','Latex')
    axis equal
    view(-45,10)
    hold off
    set(gca,'FontSize',14,'TickLabelInterpreter','latex')

    drawnow
    %}
end

