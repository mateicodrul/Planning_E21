function gazeboMoveDrone(pubTrajectory, poseRef, params)
    % Construct pose reference message
    poseRefQuat = poseEul2Quat(poseRef);
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
    pose_ref_msg.Points.TimeFromStart = rosduration(0,'DataFormat','struct');
    % Send messsage
    send(pubTrajectory, pose_ref_msg)
    pause(params.moveDelay);
end