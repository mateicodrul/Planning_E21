% Plot camera in world frame
function showCamera(pose)
    RW_C = Rzyx(pose(6), pose(5), pose(4));
    camera = rigid3d((RW_C * Rzyx(0,0,-pi/2) * Rzyx(0,pi/2,0)).', pose(1:3).');
    % Or camera = rigid3d((RW_C * rotx(-pi/2) * roty(pi/2)).', pose(1:3).');
    plotCamera('AbsolutePose', camera, 'Size', 0.25, 'Opacity', 0, 'Color', 'k')
end