function Ma_a = traslatedMatrix(Ma_b, ra, rb)
    H = hMatrix(ra, rb);
    Ht = transpose(H);
    Ma_a = Ht * Ma_b * H;
end

