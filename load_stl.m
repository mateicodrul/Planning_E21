clc
clear
close all

% Add paths to defined functions
function_paths = {'./STL_File_Reader', './Unused'};
addpath(function_paths{:})
% Add paths to files
file_paths = {'./Source_STL', './Source_Models'};
addpath(file_paths{:})

%% Load STL and extract binary array, pointcloud and occupancy map

% Load STL
save_flag = 0;
stl_file_name = '3DBenchy';
fv = oldStlRead(['./Source_STL/' stl_file_name '.stl']);
% Extract vertices
points = fv.vertices;

% Create alpha shape (polygons and polyhedras) from points
shp = alphaShape(points, 7);
% plot(shp);
% Select range to scan for points
min_query_range = floor(min([min(points(:,1)) min(points(:,2)) min(points(:,3))]));
max_query_range = ceil(max([max(points(:,1)) max(points(:,2)) max(points(:,3))]));
query_res = 0.25;%0.2;
query_range = (min_query_range:query_res:max_query_range).';
[X, Y, Z] = meshgrid(query_range, query_range, query_range);
% Get points within the range that arepart of the alpha shape
insideMat = inShape(shp, X, Y, Z);
% Create pointcloud out of 
X = X(insideMat);
Y = Y(insideMat);
Z = Z(insideMat);
Points = [X(:) Y(:) Z(:)];
figure
pcshow(Points)

% Create 3D occupancy grid map from the pointcloud
% Empty occupancy map
occMap = occupancyMap3D;
pose = [0 0 0 1 0 0 0];
maxrange = 100;
insertPointCloud(occMap, pose, Points, maxrange);
figure
show(occMap);

% Save
if save_flag
    if query_res >= 0.25
        save(['./Source_Models/' stl_file_name '.mat'], 'insideMat', 'Points', 'occMap')
    else
        save(['./Source_Models/' stl_file_name '_high_density.mat'], 'insideMat', 'Points', 'occMap')
    end
end

%% See original STL file as a solid object

clc
clear
close all

% Import STL
fv = oldStlRead('3DBenchy.stl');
patch(fv, 'FaceColor', [0.8 0.8 1.0],...
          'EdgeColor', 'none',...
          'FaceLighting', 'gouraud',...
          'AmbientStrength', 0.15);
% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material('dull');
% Fix the axes scaling, and set a nice view angle
axis('image');
view([-135 35]);

%{
fv = oldStlRead('3DBenchy.stl');
boundaries = (-40:1:40).'; % get the range of your data from the STL file.
[x,y,z] = meshgrid(boundaries);
P = [x(:) y(:) z(:)];
in = inMesh(fv,P);
img = reshape(in, [length(boundaries), length(boundaries), length(boundaries)]);
scatter(img)
%}