function pose = gazeboGetPose(subPose, poseRef)
    % Read the latest pose message until reference is reached within some
    % tolerance
    tol_position = 2.5e-1;
    tol_yaw = 5e-2;
    while true
        pose_msg = subPose.LatestMessage;
        poseQuat = [pose_msg.Position.X, pose_msg.Position.Y, pose_msg.Position.Z,...
                    pose_msg.Orientation.W, pose_msg.Orientation.X, pose_msg.Orientation.Y, pose_msg.Orientation.Z].';
        pose = poseQuat2Eul(poseQuat);
        if norm(pose(1:3) - poseRef(1:3), 2) <= tol_position && norm(pose(6) - poseRef(6), 2) <= tol_yaw
            break;
        end
        %disp('Pose')
    end
end

