classdef DivergenceMonitor < handle
    % DivergenceMonitor - Roll/Pitch発散検出システム
    % 真値と推定値を比較して異常な発散を検出し、エラー停止する
    
    properties (Access = private)
        truth_data
        step_count
        max_roll_error_deg
        max_pitch_error_deg
        error_threshold_deg
        consecutive_error_limit
        consecutive_error_count
        last_check_step
    end
    
    methods
        function obj = DivergenceMonitor(truth_csv_path, error_threshold_deg)
            % コンストラクタ
            % truth_csv_path: 真値CSVファイルのパス
            % error_threshold_deg: エラー判定の閾値（度）
            
            if nargin < 2
                error_threshold_deg = 10.0; % デフォルト10度
            end
            
            % 真値データを読み込み
            try
                obj.truth_data = readtable(truth_csv_path);
                fprintf('DivergenceMonitor: 真値データ読み込み完了 (%d サンプル)\n', ...
                        height(obj.truth_data));
            catch e
                error('DivergenceMonitor: 真値データの読み込みに失敗: %s', e.message);
            end
            
            % 初期化
            obj.step_count = 0;
            obj.max_roll_error_deg = 0;
            obj.max_pitch_error_deg = 0;
            obj.error_threshold_deg = error_threshold_deg;
            obj.consecutive_error_limit = 3; % 連続3回で異常判定（早期検出）
            obj.consecutive_error_count = 0;
            obj.last_check_step = 0;
            
            fprintf('DivergenceMonitor: 初期化完了 (エラー閾値: %.1f度)\n', ...
                    obj.error_threshold_deg);
        end
        
        function checkDivergence(obj, estimated_euler, step_index)
            % Roll/Pitch発散をチェック
            % estimated_euler: 推定オイラー角 [roll, pitch, yaw] (度)
            % step_index: 現在のステップ番号
            
            obj.step_count = obj.step_count + 1;
            
            % ステップ番号が真値データの範囲内かチェック
            if step_index > height(obj.truth_data)
                fprintf('警告: ステップ%d は真値データ範囲外 (最大: %d)\n', ...
                        step_index, height(obj.truth_data));
                return;
            end
            
            % 真値を取得（ラジアン）
            true_roll = obj.truth_data.roll(step_index);
            true_pitch = obj.truth_data.pitch(step_index);
            true_yaw = obj.truth_data.yaw(step_index);
            
            % 推定値（quat_lib は度を返すようになった）
            est_roll_deg = estimated_euler(1);
            est_pitch_deg = estimated_euler(2);
            est_yaw_deg = estimated_euler(3);
            
            % 真値（CSVファイルは既に度単位なのでそのまま使用）
            true_roll_deg = true_roll;
            true_pitch_deg = true_pitch;
            true_yaw_deg = true_yaw;
            
            % 角度誤差を計算（±180度で正規化）
            roll_error_deg = obj.normalizeAngleDiff(est_roll_deg - true_roll_deg);
            pitch_error_deg = obj.normalizeAngleDiff(est_pitch_deg - true_pitch_deg);
            yaw_error_deg = obj.normalizeAngleDiff(est_yaw_deg - true_yaw_deg);
            
            % 最大誤差を更新
            obj.max_roll_error_deg = max(obj.max_roll_error_deg, abs(roll_error_deg));
            obj.max_pitch_error_deg = max(obj.max_pitch_error_deg, abs(pitch_error_deg));
            
            % エラー判定
            roll_exceeds = abs(roll_error_deg) > obj.error_threshold_deg;
            pitch_exceeds = abs(pitch_error_deg) > obj.error_threshold_deg;
            
            if roll_exceeds || pitch_exceeds
                obj.consecutive_error_count = obj.consecutive_error_count + 1;
                
                % 簡潔な警告出力
                if obj.consecutive_error_count >= obj.consecutive_error_limit
                    fprintf('\n[ESKF Divergence] Step %d: Roll=%.2f° Pitch=%.2f° (threshold: %.1f°)\n', ...
                            step_index, abs(roll_error_deg), abs(pitch_error_deg), obj.error_threshold_deg);
                    error('ESKF発散検出: Roll/Pitchが異常発散しました (ステップ %d)', step_index);
                end
            else
                % エラーが収まった場合はカウンタリセット
                obj.consecutive_error_count = 0;
            end
            
            % 定期レポート（5000ステップごと）
            if mod(step_index, 5000) == 0
                fprintf('[Monitor] Step %d: Max errors Roll=%.2f° Pitch=%.2f°\n', ...
                        step_index, obj.max_roll_error_deg, obj.max_pitch_error_deg);
            end
            
            obj.last_check_step = step_index;
        end
        
        
        function angle_diff = normalizeAngleDiff(~, angle_diff)
            % 角度差を±180度の範囲に正規化
            while angle_diff > 180
                angle_diff = angle_diff - 360;
            end
            while angle_diff < -180
                angle_diff = angle_diff + 360;
            end
        end
        
        function truth_data_size = getTruthDataSize(obj)
            % 真値データのサイズを取得
            truth_data_size = size(obj.truth_data);
        end
        
        function truth_row = getTruthData(obj, step_index)
            % 指定ステップの真値データを取得
            if step_index <= height(obj.truth_data)
                truth_row = obj.truth_data(step_index, :);
            else
                truth_row = [];
            end
        end
    end
end