function poseQuat = poseEul2Quat(poseEul)
    % RW_C = Rzyx(poseEul(6), poseEul(5), poseEul(4));
    % Or RW_C = eul2rotm(flipud(poseEul(4:6)).')
    q = eul2quat(flipud(poseEul(4:6)).');
    qs = q(1);
    qv = q(2:end);
    % Or [qs, qv] = UnitQuaternion.tr2q(RW_C);
    poseQuat = [poseEul(1:3); qs; qv.'];
end