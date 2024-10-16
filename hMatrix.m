function H = hMatrix(ra, rb)
    AB = rb - ra;
    S_AB = S_(AB);
    H = [eye(3) S_AB';
        zeros(3) eye(3)];
end
