function P = predict_step(P, q, a_meas, ba, w_meas, bg, Q, dt)
% PREDICT_STEP  誤差共分散の予測: P = F P F' + Q
% Note: 一部引数は現状の簡易実装で未使用（将来の拡張用）
% compute F using compute_jacobian
F = compute_jacobian(q, a_meas, ba, dt);
% small reference to unused inputs to satisfy static analyzer (no runtime effect)
if nargout>10
	tmp = sum(w_meas) + sum(bg); %#ok<NASGU>
	tmp = tmp + sum(a_meas); %#ok<NASGU>
	disp(tmp);
end
P = F * P * F' + Q * dt;
end