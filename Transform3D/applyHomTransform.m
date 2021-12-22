function PointsB = applyHomTransform(PointsA, HB_A)
    PointsA_t = cart2hom(PointsA);
    PointsB_t = (HB_A * PointsA_t.').';
    PointsB = hom2cart(PointsB_t);
end