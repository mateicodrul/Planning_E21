function showOccupancy(occMap, pose, params, occ_check_divider)
    % Define 3D meshgrid of a box around the environment
    poseCam = getCamPose(pose, params);
    [X,Y,Z] = meshgrid(params.minXYZ(1):1/(occ_check_divider * params.mapResolution):params.maxXYZ(1),...
                       params.minXYZ(2):1/(occ_check_divider * params.mapResolution):params.maxXYZ(2),...
                       params.minXYZ(3):1/(occ_check_divider * params.mapResolution):params.maxXYZ(3));
    % Vectorize grid
    X = X(:);
    Y = Y(:);
    Z = Z(:);
    % Get occupancy at all grid points
    isOccupied = checkOccupancy(occMap, [X Y Z]);
    % Distiguish betweem free and occupied
    occ_bool_free = isOccupied == 0;
    occ_bool_occ = isOccupied == 1;
    occ_bool_unmapped = isOccupied == -1;
    % Free voxels are green, occupied voxels are red and unmapped ones are
    % grey
    occ_color_free = zeros(sum(occ_bool_free), 3);
    occ_color_occ = zeros(sum(occ_bool_occ), 3);
    occ_color_unmapped = zeros(sum(occ_bool_unmapped), 3);
    occ_color_free(:,2) = 255;
    occ_color_occ(:,1) = 255;
    occ_color_unmapped(:,1) = 128; occ_color_unmapped(:,2) = 0; occ_color_unmapped(:,3) = 228;

    figure(3)
    scatter3(X(occ_bool_free), Y(occ_bool_free), Z(occ_bool_free), 60, occ_color_free, 's', 'filled')
    hold on
    scatter3(X(occ_bool_occ), Y(occ_bool_occ), Z(occ_bool_occ), 60, occ_color_occ, 's', 'filled')
    %scatter3(X(occ_bool_unmapped), Y(occ_bool_unmapped), Z(occ_bool_unmapped), 60, occ_color_unmapped,'s','filled')
    title('Discovered Occupancy Map $\hat{\mathcal{M}}_{\mbox{occ}}$','Interpreter','Latex')
    axis equal
    view(-45,10)
    hold off
    set(gca,'FontSize',14,'TickLabelInterpreter','latex')
    drawnow
end
