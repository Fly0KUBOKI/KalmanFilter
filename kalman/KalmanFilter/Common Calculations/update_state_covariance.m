function [x_upd, P_upd] = update_state_covariance(x_pred, P_pred, K, H, y, R)
% Standard update with Joseph form for P for numerical stability
x_upd = x_pred + K * y;
I = eye(size(P_pred));
% If R provided, use full Joseph form; otherwise fall back to simplified symmetric update
if exist('R','var') && ~isempty(R)
    P_upd = (I - K*H) * P_pred * (I - K*H)' + K * R * K';
else
    P_upd = (I - K*H) * P_pred;
end
% enforce symmetry
P_upd = (P_upd + P_upd') / 2;
end
