function HW_B = HFromPose(pose)
    RW_B = Rzyx(pose(6), pose(5), pose(4));
    tW_WB = pose(1:3);
    HW_B = H(RW_B, tW_WB);
end