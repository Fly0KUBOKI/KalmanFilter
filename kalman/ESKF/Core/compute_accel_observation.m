function z_pred = compute_accel_observation(q, g)
    % COMPUTE_ACCEL_OBSERVATION  加速度計の観測予測値を計算
    % クォータニオンから期待される加速度計出力を計算する
    %
    % 入力:
    %   q - 姿勢クォータニオン (4x1)
    %   g - 重力ベクトル (3x1, NED座標系で [0; 0; -9.81])
    %
    % 出力:
    %   z_pred - 加速度計出力予測 (x, y成分のみ) (2x1)
    
    % 回転行列を計算
    Rb = QuaternionLib.to_rotation_matrix(q);
    
    % 重力をボディ座標系に変換
    g_body = Rb' * g;
    
    % 加速度計は -g (上向き) を測定
    a_pred = -g_body;
    
    % x, y成分のみ返す (Yaw不可観測のため)
    z_pred = a_pred(1:2);
end
