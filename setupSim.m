clc
clear
close all
color = colororder;

% Add paths to defined functions
function_paths = {'./Camera', './Transform3D', './Mapping',...
                  genpath('./Planning'), './Visualization'};
addpath(function_paths{:})

%% Simulation parameters

% Camera
params.camMaxRange = 10;
params.camPitch = 0.1;
params.fovHorizontal = pi/2;
params.fovVertical = pi/3;
% Precompute transformation between drone's bofy (B) and camera (C) frames,
% where z_C = x_B
params.HB_C = H(Rzyx(-pi/2, 0, -(pi/2 + params.camPitch)), [0.1 0 -0.03].'); 

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
params.extensionRange = 5;
% Iteration limit for building a tree
params.treeIterations = 10;
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

%% ROS setup

% Make sure Gazebo is running
% Start ROS node
rosinit('NodeName','/test_node')

% Create publisher and subscribers to send/receive messages over the ROS
% network. Use ROS messages as structures, when possible.

% Publisher
poseRef = [0 0 1 0 0 0].';
poseRefQuat = poseEul2Quat(poseRef);
pubTrajectory = rospublisher('/pelican/command/trajectory','trajectory_msgs/MultiDOFJointTrajectory','DataFormat','struct');
pose_ref_msg = rosmessage(pubTrajectory);
pose_ref_msg.Header.Stamp = rostime('now','DataFormat','struct');
pose_ref_msg.JointNames = {'base_link'};
pose_ref_msg.Points(1).MessageType = 'trajectory_msgs/MultiDOFJointTrajectoryPoint';
pose_ref_msg.Points.Transforms.Translation.X = poseRefQuat(1);
pose_ref_msg.Points.Transforms.Translation.Y = poseRefQuat(2);
pose_ref_msg.Points.Transforms.Translation.Z = poseRefQuat(3);
pose_ref_msg.Points.Transforms.Rotation.X = poseRefQuat(5);
pose_ref_msg.Points.Transforms.Rotation.Y = poseRefQuat(6);
pose_ref_msg.Points.Transforms.Rotation.Z = poseRefQuat(7);
pose_ref_msg.Points.Transforms.Rotation.W = poseRefQuat(4);
pose_ref_msg.Points.Velocities = struct([]);
pose_ref_msg.Points.Accelerations = struct([]);
pose_ref_msg.Points.TimeFromStart = rosduration (0,'DataFormat','struct');

% Subscribers
subPose = rossubscriber('/pelican/ground_truth/pose','geometry_msgs/Pose','DataFormat','struct');
subLeft = rossubscriber('/pelican/vi_sensor/left/image_raw','sensor_msgs/Image','DataFormat','struct');
subRight = rossubscriber('/pelican/vi_sensor/right/image_raw','sensor_msgs/Image','DataFormat','struct');
subDisparity = rossubscriber('/pelican/vi_sensor/camera_depth/depth/disparity','sensor_msgs/Image','DataFormat','struct');
subPointcloud = rossubscriber('/pelican/vi_sensor/camera_depth/depth/points','sensor_msgs/PointCloud2'); 
subCameraInfo = rossubscriber('/pelican/vi_sensor/camera_depth/depth/camera_info','sensor_msgs/CameraInfo','DataFormat','struct'); 
% Wait for subscribers to register with the master
pause(1);