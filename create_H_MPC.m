function H = create_H_MPC(A, N)
    H = NaN(7*N, 7);
    H(1:7, :) = A;

    for k = 1:N-1
        H(k*7+1:k*7+7, :) = A^(k+1);
    end
end

    