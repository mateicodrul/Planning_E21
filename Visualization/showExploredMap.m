function showExploredMap(PointsC_visible, occMap, pose, posePrev)
    % Extrinsics from pose
    HC_W = extrinsicsFromPose(pose);
    % Colors
    color = colororder;

    figure(1)
    set(gcf,'units','centimeters','position',[51,0,51,25.5])

    % Plot visible point cloud in world frame, on top of the full pointcloud
    subplot(1,3,1)
    if ~isempty(PointsC_visible)
        Points_visible = applyHomTransform(PointsC_visible, invH(HC_W));
        pcshow(Points_visible, color(2,:), 'BackgroundColor', [1 1 1], 'MarkerSize', 50)
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
    
    % Plot voxel map
    subplot(1,3,2)
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
    
    drawnow
end

