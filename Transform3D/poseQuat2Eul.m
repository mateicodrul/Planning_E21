function poseEul = poseQuat2Eul(poseQuat)
    eul_angles = flipud(quat2eul(poseQuat(4:end).').');
    poseEul = [poseQuat(1:3); eul_angles];
end
