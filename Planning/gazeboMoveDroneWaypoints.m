function gazeboMoveDroneWaypoints(pubTrajectory, poseRefVec, params) 
    % Construct pose reference message
    pose_ref_msg = rosmessage(pubTrajectory);
    pose_ref_msg.Header.Stamp = rostime('now','DataFormat','struct');
    pose_ref_msg.JointNames = {'base_link'};
    for i = 1:size(poseRefVec,1)
        poseRef = poseRefVec(i,:).';
        poseRefQuat = poseEul2Quat(poseRef);
        pose_ref_msg.Points(i).MessageType = 'trajectory_msgs/MultiDOFJointTrajectoryPoint';
        pose_ref_msg.Points(i).Transforms.Translation.X = poseRefQuat(1);
        pose_ref_msg.Points(i).Transforms.Translation.Y = poseRefQuat(2);
        pose_ref_msg.Points(i).Transforms.Translation.Z = poseRefQuat(3);
        pose_ref_msg.Points(i).Transforms.Rotation.X = poseRefQuat(5);
        pose_ref_msg.Points(i).Transforms.Rotation.Y = poseRefQuat(6);
        pose_ref_msg.Points(i).Transforms.Rotation.Z = poseRefQuat(7);
        pose_ref_msg.Points(i).Transforms.Rotation.W = poseRefQuat(4);
        pose_ref_msg.Points(i).Velocities = struct([]);
        pose_ref_msg.Points(i).Accelerations = struct([]);
        pose_ref_msg.Points(i).TimeFromStart = rosduration(0,'DataFormat','struct');
    end
    % Send messsage
    send(pubTrajectory, pose_ref_msg)
    pause(params.moveDelay);
end