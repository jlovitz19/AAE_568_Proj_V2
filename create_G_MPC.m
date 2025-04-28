function G = create_G_MPC(A, B, N)
    G = NaN(7*N, N);

    for k = 1:N
        for l = 1:N
            if l < k
                G((k-1)*7+1:7*k, l) = zeros(7, 1);
            elseif l == k
                G((k-1)*7+1:7*k, l) = B;
            else
                G((k-1)*7+1:7*k, l) = A^(k-l)*B;
            end
        end
    end
end