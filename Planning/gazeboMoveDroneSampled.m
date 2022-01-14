function gazeboMoveDroneSampled(pubTrajectory, startPoint, endPoint, params)
    distance = endPoint(1:3) - startPoint(1:3);
    yaw_direction = endPoint(6) - startPoint(6);
    if yaw_direction > pi
        yaw_direction = yaw_direction - 2 * pi;
    end
    if yaw_direction < -pi
        yaw_direction = yaw_direction + 2 * pi;
    end
    disc = min(params.dt * params.vMax / norm(distance), params.dt * params.dYawMax / abs(yaw_direction));
    assert(disc > 0)
    poseRefVec = zeros(length(0:disc:1) + 1,6);
    k = 1;
    for it = 0:disc:1
        origin = (1 - it) * startPoint(1:3) + it * endPoint(1:3);
        yaw = startPoint(6) + yaw_direction * it; 
        if yaw > pi
            yaw = yaw - 2 * pi;
        end
        if yaw < -pi
            yaw = yaw + 2 * pi;
        end
        poseRef = [origin; 0; 0; yaw];
        poseRefVec(k,:) = poseRef.';
        k = k + 1;
        gazeboMoveDrone(pubTrajectory, poseRef, params)
    end
    gazeboMoveDrone(pubTrajectory, [endPoint(1:3); 0; 0; yaw], params)
    %poseRefVec(end,:) = [endPoint(1:3); 0; 0; yaw].';
    %gazeboMoveDroneWaypoints(pubTrajectory, poseRefVec, params)
end

