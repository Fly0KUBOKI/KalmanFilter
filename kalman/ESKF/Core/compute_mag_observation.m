function z_pred = compute_mag_observation(q, m_world)
    % COMPUTE_MAG_OBSERVATION  磁気計の観測予測値を計算
    % クォータニオンから期待される磁気計出力を計算する
    %
    % 入力:
    %   q       - 姿勢クォータニオン (4x1)
    %   m_world - 地磁気ベクトル (3x1, NED座標系)
    %
    % 出力:
    %   z_pred - 磁気計出力予測 (正規化済み) (3x1)
    
    % 回転行列を計算
    Rb = QuaternionLib.to_rotation_matrix(q);
    
    % 地磁気をボディ座標系に変換
    m_body = Rb' * m_world;
    
    % 正規化
    m_norm = norm(m_body);
    if m_norm > 1e-6
        z_pred = m_body / m_norm;
    else
        z_pred = m_body;
    end
end
