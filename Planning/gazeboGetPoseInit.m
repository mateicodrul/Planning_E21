function pose = gazeboGetPoseInit(subPose)
    pose_msg = subPose.LatestMessage;
    poseQuat = [pose_msg.Position.X, pose_msg.Position.Y, pose_msg.Position.Z,...
                pose_msg.Orientation.W, pose_msg.Orientation.X, pose_msg.Orientation.Y, pose_msg.Orientation.Z].';
    pose = poseQuat2Eul(poseQuat);
end

