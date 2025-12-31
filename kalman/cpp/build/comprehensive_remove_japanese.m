function comprehensive_remove_japanese()
    % Comprehensive removal of Japanese comments from all MEX files
    
    files = {
        'c:\Users\takut\OneDrive\ドキュメント\MATLAB\KalmanFilter\kalman\cpp\Inc\MEX\mex_run_eskf_sensor_updates.hpp'
    };
    
    replacements = {
        % Line 17
        '前方宣言', 'Forward declaration'
        % Line 74
        '前回のセンサー値', 'Previous sensor values'
        % Line 117  
        '// input_update_gps, input_noise_gps用', '// for input_update_gps, input_noise_gps'
        % Line 121
        '// State変換', '// State conversion'
        % Line 156
        '// デバッグ出力変換', '// Debug output conversion'
        % Line 158
        '// フィールド数が12（innov, dxを含む）の場合のみ処理', '// Process only if field count >= 10 (includes innov, dx)'
        % Line 251
        'センサー更新処理（mexCallMATLAB実装）', 'Sensor update processing (mexCallMATLAB implementation)'
        % Line 388
        'GPS更新処理（mexCallMATLAB実装）', 'GPS update processing (mexCallMATLAB implementation)'
        % Line 458
        'センサー更新の内部処理（mex_eskf_do_updateのhandle_update関数を統合）', 'Internal sensor update processing (integrated handle_update from mex_eskf_do_update)'
        % Line 459
        '入力: sensor_type, meas, p, v, q, ba, bg, P, g, dt, [sample]', 'Input: sensor_type, meas, p, v, q, ba, bg, P, g, dt, [sample]'
        % Line 460
        '出力: out_p, out_v, out_q, out_ba, out_bg, out_P, should_skip', 'Output: out_p, out_v, out_q, out_ba, out_bg, out_P, should_skip'
        % Line 473
        '出力バッファを初期化', 'Initialize output buffers'
        % Line 482
        'R取得 (C++ direct implementation)', 'Get R (C++ direct implementation)'
        % Line 489
        '3x3行列から対角要素を取得', 'Get diagonal elements from 3x3 matrix'
        % Line 494
        'ベクトル形式', 'Vector format'
        % Line 499
        'スカラー（baroなど）', 'Scalar (e.g., baro)'
        % Line 504
        'sensor_data構造体を構築', 'Build sensor_data struct'
        % Line 545
        'mex_params構造体', 'mex_params struct'
        % Line 577
        'センサータイプ別設定', 'Sensor type-specific settings'
        % Line 611
        'state構造体', 'state struct'
        % Line 627
        'MEUKFCore::step() 直接呼び出し（mex_meukf_step_v2を統合）', 'MEUKFCore::step() direct call (integrated mex_meukf_step_v2)'
        % Line 631
        'MATLAB構造体からC++構造体への変換', 'Conversion from MATLAB struct to C++ struct'
        % Line 634
        'MEUKFCore::step()を直接呼び出し', 'Direct call to MEUKFCore::step()'
        % Line 637
        'C++構造体からMATLAB構造体への変換', 'Conversion from C++ struct to MATLAB struct'
        % Line 645
        '互換性のため、innovフィールドを設定（既存コードが期待している）', 'Set innov field for compatibility (existing code expects it)'
        % Line 658
        'dxフィールドを計算（dx = K * y、カルマンゲイン × イノベーション）', 'Calculate dx field (dx = K * y, Kalman gain * innovation)'
        % Line 670
        'イノベーションがない場合は0', 'If no innovation, use 0'
        % Line 677
        'noise estimate更新 (C++ direct implementation)', 'noise estimate update (C++ direct implementation)'
        % Line 679
        'innov: output.last_yから直接取得', 'innov: get directly from output.last_y'
    };
    
    fname = files{1};
    fid = fopen(fname, 'r', 'n', 'UTF-8');
    content = fread(fid, '*char')';
    fclose(fid);
    
    for i = 1:2:length(replacements)
        from = replacements{i};
        to = replacements{i+1};
        content = strrep(content, from, to);
    end
    
    fid = fopen(fname, 'w', 'n', 'UTF-8');
    fprintf(fid, '%s', content);
    fclose(fid);
    
    fprintf('Comprehensive cleanup complete for %s\n', fname);
end

comprehensive_remove_japanese();
