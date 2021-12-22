clc
clear
close all

% Load points
stl_file_name = '3DBenchy';
load(['../Source_Models/' stl_file_name '.mat'])
clearvars -except Points

Points = Points(1:5000,:);

tic
kdtree = KDTreeSearcher(Points);
toc
 
% knnsearch - find indices and distances of nearest neighbors
% rangesearch - find indices of all nearest neighbors within a distance
% that you specify
tic

newPoint = [10 20 30];
index = knnsearch(kdtree, newPoint, 'k', 1);
newParent = Points(index,:)

toc