setupSim

% Make sure Gazebo is running
% Start ROS node
rosinit('NodeName','/test_node')

% Create publisher and subscribers to send/receive messages over the ROS
% network. Use ROS messages as structures, when possible.
poseRef = [2 2 2 0 0 0].';
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
subPose = rossubscriber('/pelican/ground_truth/pose','geometry_msgs/Pose','DataFormat','struct');
subLeft = rossubscriber('/pelican/vi_sensor/left/image_raw','sensor_msgs/Image','DataFormat','struct');
subRight = rossubscriber('/pelican/vi_sensor/right/image_raw','sensor_msgs/Image','DataFormat','struct');
subDisparity = rossubscriber('/pelican/vi_sensor/camera_depth/depth/disparity','sensor_msgs/Image','DataFormat','struct');
subPointcloud = rossubscriber('/pelican/vi_sensor/camera_depth/depth/points','sensor_msgs/PointCloud2'); 
subCameraInfo = rossubscriber('/pelican/vi_sensor/camera_depth/depth/camera_info','sensor_msgs/CameraInfo','DataFormat','struct'); 
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
PointsC = readXYZ(pointcloud_msg);
PointsC(any(isnan(PointsC), 2), :) = [];
poseQuat_ = [pose_msg.Position.X, pose_msg.Position.Y, pose_msg.Position.Z,...
             pose_msg.Orientation.W, pose_msg.Orientation.X, pose_msg.Orientation.Y, pose_msg.Orientation.Z].';
pose = poseQuat2Eul(poseQuat_);

poseQuat = poseEul2Quat(pose);
insertPointCloud(occMap, poseQuat, PointsC, params.camMaxRange + 1/params.mapResolution);

figure
subplot(1,2,1)
show(occMap)
colormap turbo
view(-45,10)
subplot(1,2,2)
pcshow(PointsC, 'BackgroundColor', [1 1 1])

figure
subplot(1,2,1)
imshow(image_left)
subplot(1,2,2)
imshow(image_right)