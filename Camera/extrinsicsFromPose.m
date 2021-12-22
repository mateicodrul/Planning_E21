function HC_W = extrinsicsFromPose(pose)
    RW_C = Rzyx(pose(6), pose(5), pose(4));
    tW_WC = pose(1:3);
    RC_W = RW_C.';
    tC_CW = -RC_W * tW_WC;
    HC_W = H(RC_W, tC_CW);
end