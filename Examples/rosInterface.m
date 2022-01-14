setupSim;
clearvars -except occMap params

% Create publisher and subscribers to send/receive messages over the ROS
% network. Use ROS messages as structures, when possible.
poseRef = [0 0 1 0 0 pi/4].';
poseRefQuat = poseEul2Quat(poseRef);
%{
% Publisher using C++ node on the host computer
publisher_string = ['rosrun rotors_gazebo waypoint_publisher',...
                    ' ', num2str(pose_ref(1)), ' ', num2str(pose_ref(2)),...
                    ' ', num2str(pose_ref(3)), ' ', num2str(pose_ref(6)),...
                    ' __ns:=pelican'];
system(publisher_string);
%}

% Publisher using Matlab node
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
subPose = rossubscriber('/firefly/ground_truth/pose','geometry_msgs/Pose','DataFormat','struct');
subLeft = rossubscriber('/firefly/vi_sensor/left/image_raw','sensor_msgs/Image','DataFormat','struct');
subRight = rossubscriber('/firefly/vi_sensor/right/image_raw','sensor_msgs/Image','DataFormat','struct');
subDisparity = rossubscriber('/firefly/vi_sensor/camera_depth/depth/disparity','sensor_msgs/Image','DataFormat','struct');
subPointcloud = rossubscriber('/firefly/vi_sensor/camera_depth/depth/points','sensor_msgs/PointCloud2','DataFormat','struct'); 
subCameraInfo = rossubscriber('/firefly/vi_sensor/camera_depth/depth/camera_info','sensor_msgs/CameraInfo','DataFormat','struct'); 
% Wait for subscribers to register with the master
pause(2);

%%
% Publish message and wait for the drone to move 
send(pubTrajectory, pose_ref_msg)
pause(5);

% Get the latest messages from the topics we subscribed to
pose_msg = subPose.LatestMessage;
image_left_msg = subLeft.LatestMessage;
image_right_msg = subRight.LatestMessage;
disparity_msg = subDisparity.LatestMessage;
pointcloud_msg = subPointcloud.LatestMessage;
camerainfo_msg = subCameraInfo.LatestMessage;

% Shutdown ROS node
rosshutdown

%% Decode messages and see output

image_left = reshape(image_left_msg.Data, image_left_msg.Width, image_left_msg.Height).';
image_right = reshape(image_right_msg.Data, image_right_msg.Width, image_right_msg.Height).';
PointsC = rosReadXYZ(pointcloud_msg);
PointsC(any(isnan(PointsC), 2), :) = [];
PointsB = applyHomTransform(PointsC, params.HB_C);
poseQuat_ = [pose_msg.Position.X, pose_msg.Position.Y, pose_msg.Position.Z,...
             pose_msg.Orientation.W, pose_msg.Orientation.X, pose_msg.Orientation.Y, pose_msg.Orientation.Z].';
pose = poseQuat2Eul(poseQuat_);
poseQuat = poseEul2Quat(pose);
insertPointCloud(occMap, poseQuat, PointsB, params.camMaxRange + 1/params.mapResolution);

figure
subplot(1,2,1)
show(occMap)
colormap turbo
view(-45,10)
subplot(1,2,2)
pcshow(PointsB, 'BackgroundColor', [1 1 1])

figure
subplot(1,2,1)
imshow(image_left)
subplot(1,2,2)
imshow(image_right)