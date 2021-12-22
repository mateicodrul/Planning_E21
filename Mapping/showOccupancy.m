function [outputArg1,outputArg2] = showOccupancy(pose, area)
    % Free voxels are green and occupied voxels are red
    occ_color_free = zeros(sum(occ_bool_free),3);
    occ_color_occ = zeros(sum(occ_bool_occ),3);
    occ_color_free(:,2) = 255;
    occ_color_occ(:,1) = 255;
    
    scatter3(X(occ_bool_free), Y(occ_bool_free), Z(occ_bool_free), 30, occ_color_free)
    hold on
    scatter3(X(occ_bool_occ), Y(occ_bool_occ), Z(occ_bool_occ), 30, occ_color_occ)
    showCamera(pose)
    axis equal
    hold off
end

