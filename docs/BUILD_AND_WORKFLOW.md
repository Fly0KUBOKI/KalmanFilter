# ビルド・実行ワークフロー

## 🎯 概要

効率的な開発サイクル・自動化ビルドシステム・品質保証を通じて、高い開発生産性と安定した製品品質を実現します。

## ⚡ 高速開発サイクル

### 基本フロー (5分サイクル)

```matlab
% 1. C++修正 → 2. ビルド → 3. テスト → 4. 検証
cd kalman/cpp/build
build_mex({'mex_run_eskf'});  % 対象MEXのみビルド (30秒)
clear mex                     % キャッシュクリア (必須)
cd ../..
run_simulation(42, true);     % 固定seed・データ再利用 (1分)
```

### 効率化のポイント

**選択的ビルド**: 
```matlab
% 全てリビルド (初回のみ、3分)
build_mex(); 

% 特定コンポーネントのみ (通常運用、30秒)
build_mex({'mex_run_eskf'});     % ESKF主エンジン
build_mex({'mex_meukf_step_v2'}); % MEUKF実装
```

**データ再利用**:
```matlab
% データ生成スキップで高速テスト
run_simulation(42, true);  % skip_data_gen=true → 10秒短縮
```

---

## 🏗️ 自動ビルドシステム

### `build_mex.m` — 統合ビルドマネージャー

**依存関係自動解決**:
```matlab
function build_mex(targets)
    % ソース自動収集・依存関係グラフ生成
    source_files = collect_dependencies(targets);
    include_dirs = resolve_include_paths();
    
    % プラットフォーム別コンパイルオプション
    compile_opts = get_platform_compile_options();
    
    for target = targets
        compile_mex(target, source_files, include_dirs, compile_opts);
    end
end
```

**自動依存関係収集**:
```matlab
function sources = collect_dependencies(target)
    switch target
        case 'mex_run_eskf'
            sources = {
                'MEX/mex_run_eskf.cpp',                    % メインMEX
                'MEX/mex_eskf_initializer.cpp',            % 初期化処理
                'Lib/ESKF/src/eskf_core.cpp',              % フィルタ実装  
                'Lib/ESKF/src/eskf_sensor_updates.cpp',    % センサー統合
                'Lib/ESKF/src/eskf_math.cpp',              % 数学関数
                'Lib/ESKF/src/eskf_postprocess.cpp',       % 後処理
                'Lib/Common/src/filter_mgmt.cpp',          % 共通管理
                'Lib/Common/src/Sensor/sensor_preprocessor.cpp' % センサー前処理
            };
        case 'mex_meukf_step_v2'
            sources = {
                'MEX/mex_meukf_step.cpp',
                'Lib/MEUKF/src/meukf_core.cpp'
            };
    end
end
```

### ビルド最適化・並列化

**インクリメンタルビルド**:
```matlab
function needs_rebuild = check_build_dependency(target, sources)
    mex_file = fullfile('bin', [target '.mexw64']);
    if ~exist(mex_file, 'file')
        needs_rebuild = true; return;
    end
    
    mex_time = get_file_time(mex_file);
    
    % ソースファイル更新チェック
    for src = sources
        if get_file_time(src) > mex_time
            needs_rebuild = true; return;
        end
    end
    
    % ヘッダーファイル更新チェック  
    headers = find_headers(sources);
    for hdr = headers
        if get_file_time(hdr) > mex_time
            needs_rebuild = true; return;
        end
    end
    
    needs_rebuild = false;
end
```

**並列ビルド** (複数ターゲット同時):
```matlab
function parallel_build(targets)
    if length(targets) > 1 && has_parallel_toolbox()
        parfor i = 1:length(targets)
            build_single_target(targets{i});
        end
    else
        for target = targets
            build_single_target(target);
        end
    end
end
```

### プラットフォーム対応

**Windows (Visual Studio)**:
```matlab
function opts = get_windows_compile_options()
    opts = {
        '-O',                           % 最適化有効
        '-DNDEBUG',                     % リリースビルド
        '-DWIN32',                      % Windows定義
        '-D_CRT_SECURE_NO_WARNINGS',    % MSVC警告抑制
        '-DKALMAN_NO_STANDALONE'        % MEX専用ビルド
    };
    
    % UTF-8対応 (日本語コメント対応)
    old_flags = getenv('COMPFLAGS');
    setenv('COMPFLAGS', [old_flags ' /utf-8']);
    
    % インクルードパス自動設定
    include_paths = {
        fullfile(cpp_root, 'Lib'),
        fullfile(cpp_root, 'Lib', 'Common', 'inc'),
        fullfile(cpp_root, 'Lib', 'ESKF', 'inc'),
        fullfile(cpp_root, 'Lib', 'Matrix'),
        fullfile(cpp_root, 'Lib', 'Quaternion')
    };
    
    for path = include_paths
        opts{end+1} = ['-I' path{1}];
    end
end
```

**macOS/Linux (GCC/Clang)**:
```matlab
function opts = get_unix_compile_options()
    opts = {
        '-O3',                          % 最大最適化
        '-DNDEBUG',
        '-std=c++14',                   % C++14標準
        '-ffast-math',                  % 数学関数高速化
        '-march=native'                 % CPU最適化
    };
end
```

---

## 🧪 テスト・品質保証システム

### 単体テスト (`run_simulation.m`)

**実行モード**:
```matlab
% 開発用 (高速フィードバック)
run_simulation(42, true);          % 固定seed, データ再利用
run_simulation(42, false);         % 固定seed, データ再生成  

% 検証用 (再現性確保)
for seed = [42, 123, 456, 789]
    run_simulation(seed, false);
end
```

**自動品質チェック**:
```matlab
function quality = assess_simulation_quality(results)
    % RMSE計算
    pos_rmse = sqrt(mean(sum((results.p - truth.p).^2, 1)));
    att_rmse = sqrt(mean(sum((results.euler - truth.euler).^2, 1)));
    
    % 品質評価
    quality.pos_rmse = pos_rmse;
    quality.att_rmse = att_rmse; 
    quality.pos_ok = pos_rmse < 2.0;      % 2m以下
    quality.att_ok = att_rmse < 1.0;      % 1度以下
    
    % バイアス収束確認
    final_bg_norm = norm(results.bg(:,end));
    quality.bias_converged = final_bg_norm > 0.01;  % 非ゼロ収束
    
    quality.overall = quality.pos_ok && quality.att_ok && quality.bias_converged;
    
    % 結果レポート
    fprintf('品質評価: %s\n', quality.overall ? 'PASS' : 'FAIL');
    fprintf('  Position RMSE: %.2fm (%s)\n', pos_rmse, quality.pos_ok ? 'OK' : 'NG');
    fprintf('  Attitude RMSE: %.2f° (%s)\n', att_rmse, quality.att_ok ? 'OK' : 'NG');
    fprintf('  Bias convergence: %s\n', quality.bias_converged ? 'OK' : 'NG');
end
```

### 統計的回帰テスト (`run_batch_10sets.m`)

**10seed並列実行**:
```matlab
function run_batch_10sets()
    seeds = [42, 123, 456, 789, 101112, 131415, 161718, 192021, 222324, 252627];
    results = cell(10, 1);
    
    % 並列実行 (Parallel Computing Toolbox利用可能時)
    if has_parallel_toolbox()
        parfor i = 1:10
            results{i} = run_single_seed(seeds(i));
        end
    else
        for i = 1:10
            results{i} = run_single_seed(seeds(i));
        end
    end
    
    % 統計解析・レポート生成
    summary = analyze_batch_results(results);
    save_batch_report(summary);
end
```

**統計解析**:
```matlab
function summary = analyze_batch_results(results)
    pos_rmse_list = zeros(10, 1);
    att_rmse_list = zeros(10, 1);
    success_list = false(10, 1);
    
    for i = 1:10
        quality = assess_simulation_quality(results{i});
        pos_rmse_list(i) = quality.pos_rmse;
        att_rmse_list(i) = quality.att_rmse;
        success_list(i) = quality.overall;
    end
    
    % 統計サマリー
    summary.success_rate = sum(success_list) / 10 * 100;  % パーセント
    summary.pos_rmse_mean = mean(pos_rmse_list);
    summary.pos_rmse_std = std(pos_rmse_list);
    summary.pos_rmse_max = max(pos_rmse_list);
    summary.att_rmse_mean = mean(att_rmse_list);
    summary.att_rmse_std = std(att_rmse_list);
    summary.att_rmse_max = max(att_rmse_list);
    
    % 品質判定
    summary.overall_pass = summary.success_rate >= 90.0;  % 90%以上成功
    
    % 統計レポート出力
    fprintf('\n=== 10-Seed統計テスト結果 ===\n');
    fprintf('成功率: %.1f%% (%d/10)\n', summary.success_rate, sum(success_list));
    fprintf('Position RMSE: %.2f±%.2f m (max: %.2f)\n', ...
        summary.pos_rmse_mean, summary.pos_rmse_std, summary.pos_rmse_max);
    fprintf('Attitude RMSE: %.2f±%.2f ° (max: %.2f)\n', ...
        summary.att_rmse_mean, summary.att_rmse_std, summary.att_rmse_max);
    fprintf('総合判定: %s\n', summary.overall_pass ? 'PASS' : 'FAIL');
end
```

### 継続的品質監視

**性能劣化検知**:
```matlab
function check_performance_regression()
    % 基準性能読み込み (過去の良好な結果)
    baseline_file = 'Results/baseline_performance.mat';
    if exist(baseline_file, 'file')
        baseline = load(baseline_file);
        
        % 現在結果と比較
        current_summary = load('Results/batch_10sets_results.mat');
        
        % 劣化閾値チェック
        pos_regression = current_summary.pos_rmse_mean - baseline.pos_rmse_mean;
        att_regression = current_summary.att_rmse_mean - baseline.att_rmse_mean;
        
        if pos_regression > 0.1  % 10cm以上劣化
            warning('Position RMSE劣化検知: %.2fm → %.2fm (+%.2fm)', ...
                baseline.pos_rmse_mean, current_summary.pos_rmse_mean, pos_regression);
        end
        
        if att_regression > 0.05  % 0.05度以上劣化
            warning('Attitude RMSE劣化検知: %.2f° → %.2f° (+%.2f°)', ...
                baseline.att_rmse_mean, current_summary.att_rmse_mean, att_regression);
        end
        
        % 成功率低下チェック
        success_drop = baseline.success_rate - current_summary.success_rate;
        if success_drop > 10.0  % 10%以上低下
            warning('成功率低下検知: %.1f%% → %.1f%% (%.1f%%低下)', ...
                baseline.success_rate, current_summary.success_rate, success_drop);
        end
    end
end
```

---

## 📊 パフォーマンス管理

### 実行時間計測

**詳細プロファイリング**:
```matlab
function profile_simulation()
    profile on;  % MATLABプロファイラ有効
    
    tic;
    handle = mex_run_eskf('init', obs, static_time, dt);
    init_time = toc;
    
    tic;
    for k = 1:n_samples
        mex_run_eskf('step', handle, obs, k);
    end
    step_time = toc;
    
    tic;  
    for k = 1:n_samples
        state = mex_run_eskf('get_state', handle);
    end
    getstate_time = toc;
    
    mex_run_eskf('free', handle);
    profile off;
    
    % 性能レポート
    total_time = init_time + step_time + getstate_time;
    fprintf('\n=== パフォーマンス解析 ===\n');
    fprintf('初期化時間: %.3f秒\n', init_time);
    fprintf('ステップ実行時間: %.3f秒 (%.1f Hz)\n', step_time, n_samples/step_time);
    fprintf('状態取得時間: %.3f秒\n', getstate_time);
    fprintf('総実行時間: %.3f秒 (実時間比: %.1fx)\n', total_time, params.T/total_time);
    
    % プロファイル結果表示
    profsave(profile('info'), 'Results/profile_report');
end
```

### メモリ使用量監視

```matlab
function monitor_memory_usage()
    % MEXメモリ使用量監視 
    initial_mem = get_matlab_memory();
    
    % シミュレーション実行
    run_simulation(42, true);
    
    peak_mem = get_matlab_memory();
    mem_increase = peak_mem - initial_mem;
    
    fprintf('メモリ使用量: +%.1f MB\n', mem_increase / 1024^2);
    
    % メモリリーク検知
    clear mex;  % MEXリソース解放
    pause(1);   % ガベージコレクション待ち
    
    final_mem = get_matlab_memory(); 
    mem_leak = final_mem - initial_mem;
    
    if mem_leak > 10*1024^2  % 10MB以上残存
        warning('メモリリーク検知: %.1f MB未解放', mem_leak / 1024^2);
    end
end

function mem_bytes = get_matlab_memory()
    [~, sys] = memory;
    mem_bytes = sys.PhysicalMemory.Total - sys.PhysicalMemory.Available;
end
```

---

## 🔧 デバッグ・トラブルシューティング

### 一般的な問題と解決法

**問題1: MEXビルドエラー**
```matlab
% 症状: コンパイルエラー、リンカエラー
% 解決:
clear mex;                    % 古いMEXキャッシュクリア
cd kalman/cpp/build;
build_mex();                  % 全体リビルド
```

**問題2: 数値発散・NaN発生**
```matlab
% 症状: Position RMSE が異常に大きい、NaN出力
% 解決:
% 1. 四元数正規化確認
% 2. 共分散対称性確認  
% 3. 初期化パラメータ調整
params = config_params();
params.static_time = 10.0;    % 初期化期間延長
params.noise.base.gyro_std = single(0.3);  % ノイズレベル調整
```

**問題3: 型エラー・MEXクラッシュ**
```matlab
% 症状: "Expected single (float) array" エラー
% 解決: 
% GPS以外のセンサーデータを single に変換
obs.ax = single(obs.ax);
obs.ay = single(obs.ay);
obs.az = single(obs.az);
% GPS座標は double で保持
obs.lat = double(obs.lat);  
obs.lon = double(obs.lon);
```

### 高度なデバッグ手法

**MEX内部状態ダンプ**:
```matlab
% デバッグ情報付きビルド
build_mex_debug();            % DEBUG フラグ有効

% 詳細ログ出力
handle = mex_run_eskf('init', obs, static_time, dt);
mex_run_eskf('set_debug_level', handle, 2);  % 最大詳細度

% ステップバイステップ確認
for k = 1:10  % 最初の10ステップのみ
    mex_run_eskf('step', handle, obs, k);
    state = mex_run_eskf('get_state', handle);
    
    % 異常値チェック
    if any(isnan(state.p)) || any(isnan(state.v))
        fprintf('NaN detected at step %d\n', k);
        break;
    end
    
    % 四元数正規性確認
    qnorm = norm(state.q);
    if abs(qnorm - 1.0) > 1e-4
        fprintf('Quaternion unnormalized at step %d: norm=%.6f\n', k, qnorm);
    end
end
```

**センサーデータ検証**:
```matlab
function validate_sensor_data(obs)
    % データ範囲チェック
    assert(all(abs(obs.ax) < 50), '加速度データ異常: 範囲外');
    assert(all(abs(obs.wx) < 1000), 'ジャイロデータ異常: 範囲外');  
    
    % NaN/Inf チェック
    assert(all(isfinite(obs.ax)), '加速度データにNaN/Inf');
    assert(all(isfinite(obs.lat)), 'GPS緯度データにNaN/Inf');
    
    % 型チェック
    assert(isa(obs.ax, 'single'), '加速度データはsingle型である必要があります');
    assert(isa(obs.lat, 'double'), 'GPS座標はdouble型である必要があります');
    
    fprintf('センサーデータ検証: PASS\n');
end
```

### ログ・診断情報管理

**構造化ログ**:
```matlab
function setup_logging()
    % ログディレクトリ準備
    log_dir = fullfile('Results', 'log');
    if ~exist(log_dir, 'dir'), mkdir(log_dir); end
    
    % タイムスタンプ付きログファイル
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    log_file = fullfile(log_dir, sprintf('debug_log_%s.txt', timestamp));
    
    % ログ設定
    diary(log_file);
    fprintf('=== デバッグセッション開始: %s ===\n', datestr(now));
    
    % システム情報
    fprintf('MATLAB Version: %s\n', version);
    fprintf('Platform: %s\n', computer);
    
    % MEX情報  
    mex_files = dir('kalman/cpp/bin/*.mexw64');
    fprintf('MEXファイル数: %d\n', length(mex_files));
    for i = 1:length(mex_files)
        fprintf('  %s (%s)\n', mex_files(i).name, mex_files(i).date);
    end
end
```

**自動診断レポート**:
```matlab
function generate_diagnostic_report()
    report_file = fullfile('Results', 'diagnostic_report.txt');
    fid = fopen(report_file, 'w');
    
    fprintf(fid, '=== KalmanFilter診断レポート ===\n');
    fprintf(fid, '生成日時: %s\n\n', datestr(now));
    
    % 環境情報
    fprintf(fid, '[環境情報]\n');
    fprintf(fid, 'MATLAB: %s\n', version);
    fprintf(fid, 'コンピュータ: %s\n', computer);
    
    % ビルド情報
    fprintf(fid, '\n[ビルド情報]\n');
    build_log = dir('kalman/cpp/build/build_mex_log_*.txt');
    if ~isempty(build_log)
        [~, idx] = max([build_log.datenum]);
        latest_build = build_log(idx);
        fprintf(fid, '最新ビルド: %s\n', latest_build.date);
    end
    
    % 実行結果
    fprintf(fid, '\n[実行結果]\n');
    result_files = dir('Results/estimation_*.csv');
    fprintf(fid, '結果ファイル数: %d\n', length(result_files));
    
    if exist('Results/batch_10sets_results.mat', 'file')
        summary = load('Results/batch_10sets_results.mat');
        fprintf(fid, '最新バッチテスト成功率: %.1f%%\n', summary.success_rate);
        fprintf(fid, 'Position RMSE平均: %.2fm\n', summary.pos_rmse_mean);
    end
    
    fclose(fid);
    fprintf('診断レポートを生成しました: %s\n', report_file);
end
```