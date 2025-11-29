function update_to_kalman_compute()
% UPDATE_TO_KALMAN_COMPUTE  QuaternionLib/RotationLib呼び出しをKalmanComputeに自動変換
% 
% このスクリプトは全MATLABファイルを検索し、QuaternionLib/RotationLibの
% 呼び出しをKalmanComputeの対応する関数に置き換えます。
%
% 使用方法:
%   update_to_kalman_compute()

% マッピングテーブル（旧→新）
mappings = {
    'QuaternionLib.multiply',            'KalmanCompute.quat_multiply';
    'QuaternionLib.normalize',           'KalmanCompute.quat_normalize';
    'QuaternionLib.to_rotation_matrix',  'KalmanCompute.quat_to_rotation_matrix';
    'QuaternionLib.from_euler',          'KalmanCompute.quat_from_euler';
    'QuaternionLib.to_euler',            'KalmanCompute.quat_to_euler';
    'QuaternionLib.small_angle_quat',    'KalmanCompute.quat_small_angle';
    'QuaternionLib.integrate',           'KalmanCompute.quat_integrate';
    'QuaternionLib.conjugate',           'KalmanCompute.quat_conjugate';
    'QuaternionLib.rotate_vector',       'KalmanCompute.quat_rotate_vector';
    'QuaternionLib.to_axis_angle',       'KalmanCompute.quat_to_axis_angle';
    'QuaternionLib.from_axis_angle',     'KalmanCompute.quat_from_axis_angle';
    'QuaternionLib.slerp',               'KalmanCompute.quat_slerp';
    'RotationLib.skew_symmetric',        'KalmanCompute.rot_skew_symmetric';
    'RotationLib.from_euler',            'KalmanCompute.rot_from_euler';
    'RotationLib.to_euler',              'KalmanCompute.rot_to_euler';
    'RotationLib.orthonormalize',        'KalmanCompute.rot_orthonormalize';
    'RotationLib.rodrigues',             'KalmanCompute.rot_rodrigues';
    'RotationLib.from_two_vectors',      'KalmanCompute.rot_from_two_vectors';
};

% 検索対象ディレクトリ
search_dirs = {
    'ESKF';
    'Common';
    'EKF';
    'UKF';
    'KF';
};

total_files = 0;
total_replacements = 0;

fprintf('QuaternionLib/RotationLib -> KalmanCompute 自動変換を開始...\n\n');

for i = 1:length(search_dirs)
    dir_name = search_dirs{i};
    fprintf('ディレクトリ %s を処理中...\n', dir_name);
    
    % .mファイルを再帰的に検索
    files = dir(fullfile(dir_name, '**', '*.m'));
    
    for j = 1:length(files)
        file_path = fullfile(files(j).folder, files(j).name);
        
        % ファイルを読み込み
        try
            content = fileread(file_path);
            original_content = content;
            file_modified = false;
            
            % 各マッピングを適用
            for k = 1:size(mappings, 1)
                old_func = mappings{k, 1};
                new_func = mappings{k, 2};
                
                if contains(content, old_func)
                    content = strrep(content, old_func, new_func);
                    file_modified = true;
                end
            end
            
            % 変更があった場合のみ書き込み
            if file_modified
                fid = fopen(file_path, 'w', 'n', 'UTF-8');
                fwrite(fid, content, 'char');
                fclose(fid);
                
                count = sum(contains(mappings(:,1), old_func));
                fprintf('  ✓ %s (%d箇所置換)\n', files(j).name, count);
                total_files = total_files + 1;
                total_replacements = total_replacements + count;
            end
        catch ME
            fprintf('  ✗ %s でエラー: %s\n', files(j).name, ME.message);
        end
    end
end

fprintf('\n変換完了:\n');
fprintf('  処理ファイル数: %d\n', total_files);
fprintf('  総置換箇所数: %d\n', total_replacements);
fprintf('\n注意: common_lib_mex.m など一部のファイルは手動確認が必要です。\n');

end
