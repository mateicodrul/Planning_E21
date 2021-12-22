function homog_transform = H(R, t)
    homog_transform = [R t; zeros(1,3) 1];
end

