function wrapped_angle = wrap_to_pi(angle)
% WRAP_TO_PI - 角度を[-π, π]の範囲に正規化
% wrapToPi関数の代替実装
%
% Input:
%   angle: 角度 [rad]
%
% Output:
%   wrapped_angle: [-π, π]に正規化された角度 [rad]

wrapped_angle = mod(angle + pi, 2*pi) - pi;

end