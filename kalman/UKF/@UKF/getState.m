function [x, P] = getState(obj)
    % GETSTATE  状態と共分散を取得
    x = obj.x;
    P = obj.P;
end
