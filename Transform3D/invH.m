function Hinverse = invH(H)
    Hinverse = zeros(4,4);
    Hinverse(1:3,1:3) = H(1:3,1:3).';
    Hinverse(1:3,4) = -H(1:3,1:3).' * H(1:3,4);
    Hinverse(4,4) = 1;
end