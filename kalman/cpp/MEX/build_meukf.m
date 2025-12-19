% MEUKFの MEXを個別にビルド
fprintf('=== MEUKF MEX ビルド ===\n');

try
    % パス設定
    mex_src = 'mex_meukf_step.cpp';
    core_src = '../MEUKF/meukf_core.cpp';
    
    % インクルードパス
    inc_kf = '-I../include/KF';
    inc_common = '-I../include/Common';
    inc_meukf = '-I../MEUKF';
    
    % コンパイルオプション
    opts = {'-O', '-DNDEBUG', '-DWIN32', '-D_CRT_SECURE_NO_WARNINGS'};
    
    % 出力名
    output_name = 'mex_meukf_step_v2';
    
    % ビルド
    fprintf('コンパイル中...\n');
    mex(opts{:}, inc_kf, inc_common, inc_meukf, '-output', output_name, mex_src, core_src);
    
    % binにコピー
    src_file = [output_name '.' mexext];
    dst_file = ['../bin/' src_file];
    
    if exist(src_file, 'file')
        copyfile(src_file, dst_file, 'f');
        fprintf('成功: %s\n', dst_file);
    else
        fprintf('エラー: 出力ファイルが生成されませんでした\n');
    end
    
catch ME
    fprintf('ビルドエラー:\n');
    fprintf('%s\n', ME.message);
    fprintf('\nStack trace:\n');
    for k = 1:length(ME.stack)
        fprintf('  %s (line %d)\n', ME.stack(k).name, ME.stack(k).line);
    end
end
