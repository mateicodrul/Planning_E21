function poseQuat = poseEul2Quat(poseEul)
    q = eul2quat(flipud(poseEul(4:6)).');
    qs = q(1);
    qv = q(2:end);
    poseQuat = [poseEul(1:3); qs; qv.'];
end