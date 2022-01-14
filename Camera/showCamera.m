% Plot camera in world frame
function showCamera(pose)
    RW_C = Rzyx(pose(6), pose(5), pose(4));
    camera = rigid3d(RW_C.', pose(1:3).');
    plotCamera('AbsolutePose', camera, 'Size', 0.12, 'Opacity', 0, 'Color', 'k')
end