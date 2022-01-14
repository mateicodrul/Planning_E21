function in_range_and_fov_idx = inRangeAndFOV(voxelsC, params)
    in_range_idx = vecnorm(voxelsC.') < params.gainRange & vecnorm(voxelsC.') > 0.2;
    q_t = params.K * voxelsC.';
    q_t_ = q_t ./ q_t(end,:);
    q = q_t_(1:2,:);
    in_fov_x_idx = q(1,:) >= 1 & q(1,:) <= params.imgWidth;
    in_fov_y_idx = q(2,:) >= 1 & q(2,:) <= params.imgHeight;
    in_fov_z_idx = q_t(3,:) > 0;
    in_range_and_fov_idx = (in_range_idx & in_fov_x_idx & in_fov_y_idx & in_fov_z_idx).';
end

