function G = create_G_MPC(A, B, N)
    G = NaN(7*N, 7*N);

    for k = 1:N
        for l = 1:N
            if l < k
                G(k, l) = 0;
            elseif l == k
                G(k, l) = B;
            else
                G(k, l) = A^(k-l)*B;
            end
        end
    end
end