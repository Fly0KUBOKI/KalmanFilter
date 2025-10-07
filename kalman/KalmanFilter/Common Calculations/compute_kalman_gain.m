function K = compute_kalman_gain(P_pred, H, S)
% Compute Kalman gain in numerically stable way
% prefer solving linear system rather than explicit inverse
PHt = P_pred * H';
% Solve PHt / S using MATLAB's stable solvers (right division)
if isempty(S)
    K = zeros(size(P_pred,1), size(H,1));
else
    K = PHt / S;
end
end
