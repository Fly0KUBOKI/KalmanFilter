% csv_test - CSV読み込みテスト

fprintf('CSV読み込みテスト開始\n');

csv_file = fullfile('..', '..', 'sim_data.csv');

try
    % CSVデータを読み取り
    fprintf('CSVデータを読み込み中...\n');
    data = readtable(csv_file);
    fprintf('読み込み完了: %d行, %d列\n', height(data), width(data));
    
    % 列名確認
    fprintf('列名:\n');
    disp(data.Properties.VariableNames);
    
    % 最初の数行を表示
    fprintf('最初の3行:\n');
    disp(data(1:3, :));
    
    % 必要な列の存在確認
    required_cols = {'t', 'x', 'y', 'vx', 'vy', 'accel3_x', 'gyro3_x', 'mag3_x', 'gps_x', 'baro'};
    missing_cols = {};
    
    for i = 1:length(required_cols)
        if ~any(strcmp(data.Properties.VariableNames, required_cols{i}))
            missing_cols{end+1} = required_cols{i};
        end
    end
    
    if ~isempty(missing_cols)
        fprintf('不足している列: %s\n', strjoin(missing_cols, ', '));
    else
        fprintf('必要な列はすべて存在します\n');
    end
    
    fprintf('CSV読み込みテスト完了\n');
    
catch ME
    fprintf('エラー: %s\n', ME.message);
    fprintf('行: %d\n', ME.stack(1).line);
end