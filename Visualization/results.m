clc
clear
close all

color = colororder;
filename = ['boat1.mat'];
load(['Logging/' filename]);

disp(filename)
disp(['Execution time per iteration (s): ', num2str(time_vect(end)/length(time_vect))]);
disp(['Volume explored (%): ', num2str(1 - mapping_progress_vect(end,3))]);
disp(['params.maxXYZ: ', num2str(params.maxXYZ.')]);
disp(['params.mapResolution: ', num2str(params.mapResolution)]);
disp(['params.extensionRange: ', num2str(params.extensionRange)]);
disp(['params.treeIterations: ', num2str(params.treeIterations)]);
disp(['params.gainRange: ', num2str(params.gainRange)]);
disp(['params.gainZero: ', num2str(params.gainZero)]);
disp(['params.minVUnmapped: ', num2str(params.minVUnmapped)]);

if mapping_progress_vect(end,3) <= params.minVUnmapped
    disp('Exploration terminated due to volume');
else
    disp('Exploration terminated due to gain');
end

%%% Plot voxel map %%%
figure(1)
%set(gcf,'units','centimeters','position',[51,0,51,25.5])
set(gcf,'units','centimeters','position',[0,12.5,25,12.5])
occAx = show(occMap);
colormap turbo
hold on
posePrev = pose_vect(1,:).';
pathColor = [1 0 1];
distance_travelled = 0;
for poseIdx = 2:size(pose_vect,1)
    pose = pose_vect(poseIdx,:).';
    plot3([posePrev(1) pose(1)], [posePrev(2) pose(2)], [posePrev(3) pose(3)],...
          'LineStyle', '-', 'LineWidth', 1.5, 'Color', pathColor,...
          'Marker', 'o', 'MarkerSize', 4,...
          'MarkerFaceColor', pathColor, 'MarkerEdgeColor', pathColor)
    distance_travelled = distance_travelled + norm(pose(1:3) - posePrev(1:3),2);
    posePrev = pose;
end
title('Discovered Voxel Map $\mathcal{M}$','Interpreter','Latex')
xlabel('$x_{\mathcal{W}}$ [m]','Interpreter','Latex');
ylabel('$y_{\mathcal{W}}$ [m]','Interpreter','Latex');
zlabel('$z_{\mathcal{W}}$ [m]','Interpreter','Latex');
legend_handle = legend('','Planned Path');
set(legend_handle,'Interpreter','Latex','Location','best','FontSize',14);
view(45,10)
axis equal
grid on
set(gca,'FontSize',14,'TickLabelInterpreter','latex')

disp(['Distance travelled: ', num2str(distance_travelled)]);

%% Occupancy of final map

% Define 3D meshgrid of a box around the environment
occ_check_divider = 1;
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
occ_color_occ(:,1) = color(2,1); occ_color_occ(:,2) = color(2,2); occ_color_occ(:,3) = color(2,3);
occ_color_unmapped(:,1) = color(1,1); occ_color_unmapped(:,2) = color(1,2); occ_color_unmapped(:,3) = color(1,3);

figure(2)
set(gcf,'units','centimeters','position',[0,12.5,25,12])
%subplot(1,2,1)
%scatter3(X(occ_bool_free), Y(occ_bool_free), Z(occ_bool_free), 60, occ_color_free, 's', 'filled')
hold on
scatter3(X(occ_bool_occ), Y(occ_bool_occ), Z(occ_bool_occ), 60, occ_color_occ, 's', 'filled')
scatter3(X(occ_bool_unmapped), Y(occ_bool_unmapped), Z(occ_bool_unmapped), 60, occ_color_unmapped, 's', 'filled')
title('Occupied and Unmapped Voxels in $\mathcal{M}$','Interpreter','Latex')
axis equal
view(46,10)
grid on
hold off
xlabel('$x_{\mathcal{W}}$ [m]','Interpreter','Latex');
ylabel('$y_{\mathcal{W}}$ [m]','Interpreter','Latex');
zlabel('$z_{\mathcal{W}}$ [m]','Interpreter','Latex');
legend_handle = legend('$\mathcal{M}_{occ}$','$\mathcal{M}_{unm}$');
set(legend_handle,'Interpreter','Latex','Location','best','FontSize',14);
set(gca,'FontSize',14,'TickLabelInterpreter','latex')

%% Mapping progress

figure(3)
set(gcf,'units','centimeters','position',[25,12.5,15,10])
%subplot(1,2,2)
plot(time_vect, mapping_progress_vect(:,1),...
     'LineStyle', '-', 'LineWidth', 1.5, 'Color', color(1,:))
hold on
plot(time_vect, mapping_progress_vect(:,2),...
     'LineStyle', '-', 'LineWidth', 1.5, 'Color', color(2,:))
plot(time_vect, mapping_progress_vect(:,3),...
     'LineStyle', '-', 'LineWidth', 1.5, 'Color', color(3,:))
grid on
xlim([0 time_vect(end)]);
ylim([0 1]);
xlabel('Time [s]','Interpreter','Latex');
ylabel('Volume [\%]','Interpreter','Latex');
legend_handle = legend('$\mathcal{M}_{free}$','$\mathcal{M}_{occ}$','$\mathcal{M}_{unm}$');
set(legend_handle,'Interpreter','Latex','Location','best','FontSize',14);
set(gca,'FontSize',14,'TickLabelInterpreter','latex')
