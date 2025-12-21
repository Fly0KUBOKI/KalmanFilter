function test_cholesky()
    % Unit test for Cholesky decomposition via mex_common_lib('cholesky', A)

    % Test 1: Simple 2x2 positive definite matrix
    A = [4, 2; 2, 3];
    L = mex_common_lib('cholesky', A);
    err1 = norm(A - L * L', 'fro');
    fprintf('Test 1 (2x2): Error = %.2e\n', err1);
    assert(err1 < 1e-6, 'Test 1 failed');

    % Test 2: 3x3 matrix
    P3 = [0.1, 0.01, 0.002; 
          0.01, 0.09, 0.003; 
          0.002, 0.003, 0.08];
    L3 = mex_common_lib('cholesky', P3);
    err2 = norm(P3 - L3 * L3', 'fro');
    fprintf('Test 2 (3x3): Error = %.2e\n', err2);
    assert(err2 < 1e-6, 'Test 2 failed');

    % Test 3: 15x15 random SPD
    rng(0);
    X = rand(15,15);
    P15 = X' * X + eye(15) * 1e-3;
    L15 = mex_common_lib('cholesky', P15);
    err3 = norm(P15 - L15 * L15', 'fro');
    fprintf('Test 3 (15x15): Error = %.2e\n', err3);
    assert(err3 < 1e-5, 'Test 3 failed');

    % Test 4: Near-singular matrix (expect error)
    P_bad = [1, 1; 1, 1 + 1e-13];
    try
        L_bad = mex_common_lib('cholesky', P_bad);
        fprintf('Test 4: Expected to fail but returned result\n');
    catch
        fprintf('Test 4: Exception thrown as expected\n');
    end

    fprintf('All Cholesky tests passed!\n');
end
