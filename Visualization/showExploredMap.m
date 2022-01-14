function showExploredMap(occMap, pose, posePrev, PointsC, params)
    color = colororder;

    figure(1)
    %set(gcf,'units','centimeters','position',[51,0,51,25.5])
    set(gcf,'units','centimeters','position',[0,12.5,25,12.5])

    %%% Plot voxel map %%%
    %subplot(1,3,2)
    show(occMap)
    colormap turbo
    hold on
    %showCamera(getCamPose(pose, params))
    plot3([posePrev(1) pose(1)], [posePrev(2) pose(2)], [posePrev(3) pose(3)],...
          'LineStyle', '-', 'LineWidth', 1.5, 'Color', color(1,:),...
          'Marker', 'o', 'MarkerSize', 1,...
          'MarkerFaceColor', color(1,:), 'MarkerEdgeColor', color(1,:))
    title('Discovered Voxel Map $\mathcal{M}$','Interpreter','Latex')
    view(-45,10)
    axis equal
    grid on
    set(gca,'FontSize',14,'TickLabelInterpreter','latex')
    
    %{
    %%% Plot visible point cloud in world frame %%%
    subplot(1,3,1)
    if ~isempty(PointsC)
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
end

