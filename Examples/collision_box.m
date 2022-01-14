startState = pose(1:3) + [0; 0.5; 0];
direction = [sqrt(3)/4; sqrt(5)/4; sqrt(7)/4];
endState = startState + direction;
epsilon = 1e-3;
disc_step_box = params.boundingBox ./ ...
                ceil((params.boundingBox + epsilon) * params.mapResolution);
disc_step_box(1:2) = 1.5 * disc_step_box(1:2);
disc_step_box(3) = 2 * disc_step_box(3);
[disc_grid_box_X, disc_grid_box_Y, disc_grid_box_Z] = meshgrid(...
            -params.boundingBox(1)/2 : disc_step_box(1) : params.boundingBox(1)/2, ...
            -params.boundingBox(2)/2 : disc_step_box(2) : params.boundingBox(2)/2, ...
            -params.boundingBox(3)/2 : disc_step_box(3) : params.boundingBox(3)/2);
disc_points_box = [disc_grid_box_X(:) disc_grid_box_Y(:) disc_grid_box_Z(:)];

figure
set(gcf,'units','centimeters','position',[0,0,15,15])
showCollisionBox([startState; 0; 0; 0], [endState; 0; 0; 0], params);
hold on
show(occMap)
colormap turbo
for i = 1:size(disc_points_box, 1)
    offset = disc_points_box(i,:);
    % No need to convert directions to camera frame, since we here
    % consider the coordinate frame associated with startState as being
    % parallel to the global coordinate frame, and translation should
    % not play a role, since direction is a difference between 2
    % vectors in world coordinates
    [intersection, ~] = rayIntersection(occMap, [startState.' + offset, 1, zeros(1,3)], direction.', params.camMaxRange, false);
    % If the intersection is closer than the endpoint difference,
    % it means a collision has been found along current direction, and
    % we return
    so = startState.' + offset;
    eo = endState.' + offset;
    h_ray = plot3([so(1) eo(1)], [so(2) eo(2)], [so(3) eo(3)],...
                  'LineStyle', '-', 'LineWidth', 1.5, 'Color',color(5,:));
    h_s = plot3(so(1), so(2), so(3),...
                'LineStyle', 'none', 'LineWidth', 1.5, 'Color', color(1,:),...
                'Marker', 'o', ...
                'MarkerFaceColor', color(1,:), 'MarkerEdgeColor', color(1,:));
    h_e = plot3(eo(1), eo(2), eo(3),...
                'LineStyle', 'none', 'LineWidth', 1.5, 'Color', color(4,:),...
                'Marker', 'o', ...
                'MarkerFaceColor', color(4,:), 'MarkerEdgeColor', color(4,:));
    axis equal
    set(gca, 'XTick', [], 'YTick', [], 'ZTick', [])
    title('')
    xlim([0 5])
    ylim([0 2.5])
    xlabel('$\boldmath{\mathbf{\hat{x}}}$','Interpreter','latex')
    ylabel('$\boldmath{\mathbf{\hat{y}}}$','Interpreter','latex')
    zlabel('$\boldmath{\mathbf{\hat{z}}}$','Interpreter','latex')
    set(gca,'FontSize',18,'TickLabelInterpreter','latex')
    view(-62, 10)
end
legend_handle = legend('','','','','','Cast Rays','Initial sample points','Final sample points');
set(legend_handle,'Interpreter','Latex','Location','bestoutside','FontSize',14);