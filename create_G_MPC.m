function G = create_G_MPC(A, B, N, n_x)
    G = NaN(n_x*N, N);

    for k = 1:N
        for l = 1:N
            if l < k
                G((k-1)*n_x+1:n_x*k, l) = zeros(n_x, 1);
            elseif l == k
                G((k-1)*n_x+1:n_x*k, l) = B;
            else
                G((k-1)*n_x+1:n_x*k, l) = A^(k-l)*B;
            end
        end
    end
end