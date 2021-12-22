clc
clear
close all
color = colororder;

% Add paths to defined functions
function_paths = {'./Camera', './Transform3D', './Mapping',...
                  genpath('./Planning'), './Visualization'};
addpath(function_paths{:})

% Add paths to files 
file_paths = {'./Source_Models'};
addpath(file_paths{:})

%% Load model of real environment
stl_file_name = '3DBenchy';
load(['./Source_Models/' stl_file_name '.mat'])

Points = Points./2;

%% Simulation parameters

% Camera
params.camMaxRange = 10;
params.fovHorizontal = pi/2;
params.fovVertical = pi/3;

% Occupancy Map
params.mapFreeThresh = 0.41;
params.mapOccThresh = 0.69;
% Map resolution is in cells/m. In the paper is m/cell
params.mapResolution = 4;
params.occInflationRadius = 1/params.mapResolution;

% Planning
params.minXYZ = [-35; -25; 0]/2;
params.maxXYZ = [35; 25; 50]/2;
params.boundingBox = [0.5; 0.5; 0.3];
params.collisionOvershoot = 0.5;
% Maximum length of tree branch
params.extensionRange = 2.5;
% Iteration limit for building a tree
params.treeIterations = 20;
% Iteration limit for building a tree provided the tree best gain stays 0
params.maxIterations = 200;

% Gain Computation
params.gainRange = 2;
params.degressiveCoeff = 0.25;
% Gain increments for different types of cells.
% gi stands for 'gain increment'
params.giFree = 0; 
params.giOcc = 0;
params.giUnmapped = 1;

% System dynamics
params.stepSize = 0.1;
params.vMax = 0.25;
params.dyawMax = 0.5;

%{
Unused yet
params.giProbabilistic;
params.zeroGain;
params.softBounds;

%params.focalLenght = 500;
%params.imgWidth = 1920;
%params.imgHeight = 1080;
%params.fovHorizontal = 2*atan(params.imgWidth/(2*params.focalLenght));
%params.fovVertical = 2*atan(params.imgHeight/(2*params.focalLenght));
%}

% Empty occupancy map, in which scans are incorporated iteratively.
% The occupancy map is the discovered map in world coordinates.
occMap = occupancyMap3D(params.mapResolution,...
                        'FreeThreshold', params.mapFreeThresh,...
                        'OccupiedThreshold', params.mapOccThresh);