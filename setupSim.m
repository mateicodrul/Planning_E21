clc
clear
close all
color = colororder;

% Add paths to defined functions
function_paths = {'./Camera', './Transform3D', './Mapping',...
                  genpath('./Planning'), './Visualization', './Logging'};
addpath(function_paths{:})

%% ROS setup

% IPs of ROS master and localhost (runnig local Matlab node)
masterIP = '100.67.33.73';
matlabNodeIP = '100.67.33.67';
% Make sure master is running before trying to establish a connection 
% Start local Matlab node
try
  rosinit(masterIP,'NodeHost',matlabNodeIP,'NodeName','/matlab_node')
catch ME
  rosshutdown
  rosinit(masterIP,'NodeHost',matlabNodeIP,'NodeName','/matlab_node')
end

% Create publisher and subscribers to send/receive messages over the ROS
% network. Use ROS messages as structures, when possible.
mav_namespace = '/firefly'; 

% Publisher
pubTrajectory = rospublisher([mav_namespace, '/command/trajectory'],'trajectory_msgs/MultiDOFJointTrajectory','DataFormat','struct');

% Subscribers
subPose = rossubscriber([mav_namespace, '/ground_truth/pose'],'geometry_msgs/Pose','DataFormat','struct');
subPointcloud = rossubscriber([mav_namespace, '/vi_sensor/camera_depth/depth/points'],'sensor_msgs/PointCloud2','DataFormat','struct');
subCameraInfo = rossubscriber([mav_namespace, '/vi_sensor/camera_depth/depth/camera_info'],'sensor_msgs/CameraInfo','DataFormat','struct'); 
subDepth = rossubscriber([mav_namespace, '/vi_sensor/camera_depth/depth/disparity'],'sensor_msgs/Image','DataFormat','struct'); 

% Wait for subscribers to register with the master
pause(2);

%% Simulation parameters

% Logging
logfile_name = 'duck99';

% Camera (some parameters are obtained from the camera URDF file)
camerainfo_msg = subCameraInfo.LatestMessage;
params.K = double(reshape(camerainfo_msg.K, 3, 3)).';
params.focalLength = params.K(1,1);
params.Tx = 0.11;
params.imgWidth = double(camerainfo_msg.Width);
params.imgHeight = double(camerainfo_msg.Height);
params.fovHorizontal = 2*atan(params.imgWidth/(2*params.focalLength));
params.fovVertical = 2*atan(params.imgHeight/(2*params.focalLength));
params.camMaxRange = 5;
params.camMaxDepth = params.camMaxRange + 0.1;
params.camMinDepth = 0.3;
params.camPitch = 0.261;
% Precompute transformation between drone's body (B) and camera (C) frames
RB_C = Rzyx(-pi/2, 0, -(pi/2 + params.camPitch));
tB_C = [0.1; 0; -0.03] + [0.015; 0.055; 0.0065];
params.HB_C = H(RB_C, tB_C);

% Occupancy Map
% mapFreeThresh selected such that empty space is made free
params.mapFreeThresh = 0.40;
% mapOccThresh was tuned to this value
params.mapOccThresh = 0.51;%0.55;%0.69;
% Map resolution is in cells/m. In the paper is m/cell
params.mapResolution = 1/0.125;
params.occInflationRadius = 1/params.mapResolution;%0.2%2/params.mapResolution;

% Planning
params.moveDelay = 0;
params.minXYZ = [0; 0; 0.2];
params.maxXYZ = [8; 6; 3];
params.boundingBox = [0.5; 0.5; 0.3];
params.collisionOvershoot = 0.5;
% Maximum length of tree branch
params.extensionRange = 1;
% Iteration limit for building a tree
params.treeIterations = 10;
% Iteration limit for building a tree provided the tree best gain stays 0
params.maxIterations = 100;

% Gain Computation
params.gainRange = 2;
params.degressiveCoeff = 0.5;
% Gain increments for different types of cells.
% gi stands for 'gain increment'
params.giFree = 0; 
params.giOcc = 0;
params.giUnmapped = 1;
% gainZero is the gain considered to be the threshold under which no real
% exploration happens anymore
params.gainZero = 0.1 * params.maxIterations * (1/params.mapResolution)^3 * exp(-params.degressiveCoeff*params.extensionRange);
params.minVUnmapped = params.mapResolution / 5 * 0.02;

% Visualization
params.showMap = true;
params.showRRT = false;
params.showProgress = false;

% Simulation
params.dt = 0.05;
params.vMax = 0.5;
params.dYawMax = 0.75;

%{
%%% Unused %%%
params.giProbabilistic
%}

% Empty occupancy map, in which scans are incorporated iteratively.
% The occupancy map is the discovered map in world coordinates.
occMap = occupancyMap3D(params.mapResolution,...
                        'FreeThreshold', params.mapFreeThresh,...
                        'OccupiedThreshold', params.mapOccThresh);
% Map progress logging
[voxels_X, voxels_Y, voxels_Z] = meshgrid(...
                params.minXYZ(1) : 1/params.mapResolution : params.maxXYZ(1), ...
                params.minXYZ(2) : 1/params.mapResolution : params.maxXYZ(2), ...
                params.minXYZ(3) : 1/params.mapResolution : params.maxXYZ(3));
voxels = [voxels_X(:) voxels_Y(:) voxels_Z(:)];
mapping_progress_vect = [0 0 1];
time_vect = 0; 
pose_vect = [];

% Clear unnecessary variables
clear subCameraInfo camerainfo_msg ME mav_namespace masterIP matlabNodeIP function_paths
