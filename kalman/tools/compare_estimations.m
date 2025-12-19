function compare_estimations(fileA, fileB)
    if nargin<2
        error('Usage: compare_estimations(fileA, fileB)')
    end
    A = readtable(fileA);
    B = readtable(fileB);
    % assume same length and same time
    init_samples = 2000;
    idx = init_samples+1:height(A);
    posA = [A.px(idx), A.py(idx), A.pz(idx)];
    posB = [B.px(idx), B.py(idx), B.pz(idx)];
    % compute RMSE for each file
    truth = kalman_tools_utils.read_truth_data();
    posxA = kalman_tools_utils.compute_rmse(A.px(idx), truth.x(idx));
    posyA = kalman_tools_utils.compute_rmse(A.py(idx), truth.y(idx));
    poszA = kalman_tools_utils.compute_rmse(A.pz(idx), truth.z(idx));
    posxB = kalman_tools_utils.compute_rmse(B.px(idx), truth.x(idx));
    posyB = kalman_tools_utils.compute_rmse(B.py(idx), truth.y(idx));
    poszB = kalman_tools_utils.compute_rmse(B.pz(idx), truth.z(idx));
    fprintf('RMSE comparison (X / Y / Z)\n');
    fprintf('FileA: %s -> %.4f / %.4f / %.4f\n', fileA, posxA, posyA, poszA);
    fprintf('FileB: %s -> %.4f / %.4f / %.4f\n', fileB, posxB, posyB, poszB);
    fprintf('Delta (B - A): %.4f / %.4f / %.4f\n', posxB-posxA, posyB-posyA, poszB-poszA);
end
