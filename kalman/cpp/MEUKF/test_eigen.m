function test_eigen()
    % Unit test for eigen_decompose via mex_common_lib('eigen_decompose', A)

    % Test 1: Simple 2x2 symmetric matrix
    A = [3, 1; 1, 2];
    [eig_vals, eig_vecs] = mex_common_lib('eigen_decompose', A);
    for i = 1:2
        v = eig_vecs(:, i);
        lambda = eig_vals(i, 1);
        residual = norm(A * v - lambda * v);
        fprintf('Test 1, eigenvector %d: residual = %.2e\n', i, residual);
        assert(residual < 1e-5, sprintf('Test 1.%d failed', i));
    end

    % Test 2: 3x3 matrix
    P = [0.1, 0.01, 0.002; 
         0.01, 0.09, 0.003; 
         0.002, 0.003, 0.08];
    [eig_vals, eig_vecs] = mex_common_lib('eigen_decompose', P);
    Lambda = diag(eig_vals);
    P_reconstructed = eig_vecs * Lambda * eig_vecs';
    err = norm(P - P_reconstructed, 'fro');
    fprintf('Test 2 (3x3 reconstruction): Error = %.2e\n', err);
    assert(err < 1e-5, 'Test 2 failed');

    % Test 3: 15x15
    rng(1);
    X = rand(15,15);
    P15 = (X + X')/2;
    P15 = P15 + eye(15) * (abs(min(eig(P15))) + 1e-3);
    [vals15, vecs15] = mex_common_lib('eigen_decompose', P15);
    Lambda15 = diag(vals15);
    P15_reconstructed = vecs15 * Lambda15 * vecs15';
    err15 = norm(P15 - P15_reconstructed, 'fro');
    fprintf('Test 3 (15x15 reconstruction): Error = %.2e\n', err15);
    assert(err15 < 1e-4, 'Test 3 failed');

    fprintf('All eigen decomposition tests passed!\n');
end
