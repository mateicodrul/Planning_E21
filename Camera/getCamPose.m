function poseCam = getCamPose(pose, params)
    poseCam = zeros(6,1);
    poseCam(1:3) = pose(1:3) + params.HB_C(1:3,4);
    poseCam(4) = -(pi/2 + params.camPitch);
    poseCam(5) = 0;
    poseCam(6) = pose(6) - pi/2;
end
