function H = create_H_MPC(A, N, n_x)
    H = NaN(n_x*N, n_x);
    H(1:n_x, :) = A;

    for k = 1:N-1
        H(k*n_x+1:k*n_x+n_x, :) = A^(k+1);
    end
end

    