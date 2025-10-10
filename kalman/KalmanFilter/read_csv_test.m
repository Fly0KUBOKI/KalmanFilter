% read_csv_test - read_csv_data関数のテスト

fprintf('read_csv_data関数テスト開始\n');

csv_file = fullfile('..', '..', 'sim_data.csv');

try
    fprintf('read_csv_data関数を実行中...\n');
    [sensor_data, truth_data] = read_csv_data(csv_file);
    
    fprintf('センサデータ構造体フィールド:\n');
    disp(fieldnames(sensor_data));
    
    fprintf('真値データ構造体フィールド:\n');
    disp(fieldnames(truth_data));
    
    fprintf('データサイズ確認:\n');
    fprintf('  時間データ: %d点\n', length(sensor_data.t));
    fprintf('  加速度データ: %dx%d\n', size(sensor_data.accel3));
    fprintf('  ジャイロデータ: %dx%d\n', size(sensor_data.gyro3));
    fprintf('  地磁気データ: %dx%d\n', size(sensor_data.mag3));
    fprintf('  GPSデータ: %dx%d\n', size(sensor_data.gps));
    
    fprintf('最初の数点の値:\n');
    fprintf('  時刻: %.4f, %.4f, %.4f\n', sensor_data.t(1:3));
    fprintf('  dt: %.4f, %.4f, %.4f\n', sensor_data.dt(1:3));
    fprintf('  加速度X: %.4f, %.4f, %.4f\n', sensor_data.accel3(1:3, 1));
    
    fprintf('read_csv_data関数テスト完了\n');
    
catch ME
    fprintf('エラー: %s\n', ME.message);
    fprintf('ファイル: %s\n', ME.stack(1).file);
    fprintf('行: %d\n', ME.stack(1).line);
    if length(ME.stack) > 1
        for i = 2:length(ME.stack)
            fprintf('  呼び出し元: %s (行 %d)\n', ME.stack(i).file, ME.stack(i).line);
        end
    end
end