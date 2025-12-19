function y = wrapToPi(x)
% Wrap angle(s) to [-pi, pi]
% y = wrapToPi(x)
% Works element-wise
    y = mod(x + pi, 2*pi) - pi;
end
