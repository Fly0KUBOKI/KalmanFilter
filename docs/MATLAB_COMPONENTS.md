# MATLABコンポーネント詳細

## 🎯 概要

MATLABフロントエンドは、柔軟な実験環境・直感的なデータ操作・リアルタイム可視化を提供し、C++ MEXエンジンを制御します。

## 🚀 実行エントリーポイント

### `run_simulation.m` — 単体テスト実行

**主要機能**: 単一seedでのシミュレーション実行・結果検証

```matlab
function run_simulation(seed, skip_data_gen)
% seed: ランダムシード (再現性確保)
% skip_data_gen: データ生成スキップフラグ

% 使用例
run_simulation(42, true);  % seed=42、データ再生成なし
run_simulation();          % ランダムseed、データ自動生成
```

**処理フロー**:
1. **初期化**: パス追加・MEXロード・乱数シード設定
2. **データ読込**: `GenerateData/sensor_data.csv` 取得
3. **MEX初期化**: `mex_run_eskf('init', obs, static_time, dt)`
4. **フィルタループ**: 各タイムステップで状態更新・取得
5. **結果保存**: `Results/estimation_01.csv` 出力

**主要な設定**:
- **static_time**: 5.0秒 (初期化期間、ジャイロバイアス推定用)
- **dt**: 0.0025秒 (400Hz サンプリング)
- **出力形式**: CSV (時刻・位置・速度・姿勢・バイアス)

---

### `run_batch_10sets.m` — 統計的回帰テスト

**主要機能**: 10個の異なるseedで並列実行・統計解析・品質保証

```matlab
run_batch_10sets()
% → Results/batch_10sets_results.mat
% → Results/estimation_{01-10}.csv  
% → Results/log/batch_10sets_log_YYYYMMDD_HHMMSS.txt
```

**統計指標**:
- **Position RMSE**: [0.80-0.91m] 目標値
- **Attitude RMSE**: [0.25-0.30°] 目標値  
- **成功率**: 100% (10/10) 必達条件
- **Gyro bias収束**: 非ゼロ値確認 (正常動作指標)

**品質基準**:
```matlab
% RMSE閾値 (自動判定)
pos_rmse_threshold = 2.0;  % meters
att_rmse_threshold = 1.0;  % degrees

% 異常検知
if pos_rmse > pos_rmse_threshold
    warning('Position RMSE異常: %.2fm', pos_rmse);
end
```

---

## 📊 データ生成システム (`GenerateData/`)

### `sim_generate.m` — センサーデータ生成統合

**生成パラメータ** (`config_params.m`):
```matlab
params.dt = 0.0025;              % 400Hz サンプリング
params.T = 50;                   % 50秒シミュレーション  
params.static_time = 5;          % 初期静止期間
params.motion_type = 'circular'; % 円運動パターン
```

**ノイズ・外れ値設定**:
- **加速度**: σ=0.1 m/s² + 外れ値(2%) 範囲±2.0 m/s²
- **ジャイロ**: σ=0.5 deg/s + 外れ値(2%) 範囲±2.0 deg/s  
- **磁気**: σ=5.0 nT + 外れ値(2%) 範囲±50.0 nT
- **GPS**: σ=1.0 m (lat/lon/alt) + 外れ値(2%) 範囲±10.0 m
- **気圧**: σ=1.0 m + 外れ値(2%) 範囲±5.0 m

### `generate_circular_motion.m` — 軌道生成

**円運動パラメータ**:
```matlab
radius = 50;           % 半径 50m
angular_velocity = 0.1; % 角速度 0.1 rad/s → 周期 62.8秒
```

**出力軌道**:
- **Position**: `[R*cos(ωt), R*sin(ωt), 高度一定]`
- **Velocity**: `[-Rω*sin(ωt), Rω*cos(ωt), 0]`  
- **Attitude**: 速度方向に向き調整 (`heading_mode = 'align_velocity'`)

### `add_sensor_noise.m` — リアルなセンサー特性

**高度なノイズモデル**:
- **ピンクノイズ**: 低周波ドリフト再現
- **Allan偏差**: ジャイロスコープの時間変動
- **外れ値注入**: 実環境での異常値を模擬
- **相関ノイズ**: センサー間の相互影響

---

## 🔧 MEXインターフェース管理

### `kalman/cpp/build/build_mex.m` — 自動ビルドシステム

**ビルド対象**:
```matlab
% 主要MEXターゲット
build_mex({'mex_run_eskf'});     % ESKF主エンジン
build_mex({'mex_meukf_step_v2'}); % MEUKF実装
build_mex();                     % 全て再ビルド
```

**コンパイルオプション**:
- **最適化**: `-O -DNDEBUG` (Release ビルド)
- **型安全**: `-DWIN32 -D_CRT_SECURE_NO_WARNINGS`  
- **スタンドアロン分離**: `-DKALMAN_NO_STANDALONE`
- **UTF-8対応**: `COMPFLAGS=/utf-8`

### MEX呼び出し仕様

**初期化**:
```matlab
handle = mex_run_eskf('init', obs, static_time, dt);
% obs: センサー観測構造体 (ax,ay,az,wx,wy,wz,mx,my,mz,pressure,lat,lon,alt)
% static_time: 静止初期化時間 [秒]  
% dt: サンプリング間隔 [秒]
% 戻り値: 内部状態ハンドル (uint64)
```

**ステップ実行**:
```matlab
mex_run_eskf('step', handle, obs, k);
% handle: 初期化で取得したハンドル
% obs: 現在タイムステップの観測値
% k: ステップ番号 (1-based)
```

**状態取得**:
```matlab
state = mex_run_eskf('get_state', handle);
% 戻り値構造体:
%   state.p:     位置 [single 3x1] [m]
%   state.v:     速度 [single 3x1] [m/s]  
%   state.q:     四元数 [single 4x1] [w,x,y,z]
%   state.euler: オイラー角 [single 3x1] [deg] (Roll,Pitch,Yaw)
%   state.ba:    加速度バイアス [single 3x1] [m/s²]
%   state.bg:    ジャイロバイアス [single 3x1] [rad/s]
%   state.P:     共分散行列 [single 15x15] (対称化済み)
```

**リソース解放**:
```matlab
mex_run_eskf('free', handle);
```

---

## 📈 可視化・解析システム

### `Graph/plot_csv_file.m` — 結果プロット

**プロット種別**:
- **軌道**: XY平面・3D軌道表示  
- **姿勢**: Roll/Pitch/Yaw時系列
- **バイアス**: 加速度・ジャイロバイアス収束
- **誤差**: RMSE・Innovation・Mahalanobis距離

### `FFT/` — 周波数解析

**`fft_analysis.m`**: センサー信号のスペクトル解析
```matlab
fft_analysis(sensor_data, config);
% 加速度・ジャイロ・磁気の周波数特性解析
% ノイズ・振動成分の特定
```

**`fft_lowpass_filter.m`**: ローパスフィルタ適用
```matlab
filtered_data = fft_lowpass_filter(sensor_data, config);
% カットオフ周波数: 0.1Hz (デフォルト)
% FFTドメインでの理想ローパス
```

---

## ⚙️ 設定管理システム

### `GenerateData/config_params.m` — 統一パラメータ

**主要設定**:
```matlab
function params = config_params()
    % システムパラメータ
    params.dt = 0.0025;           % サンプリング間隔
    params.T = 50;                % シミュレーション時間  
    params.static_time = 5;       % 初期化時間
    
    % 運動設定
    params.motion_type = 'circular';
    params.heading_mode = 'align_velocity';
    
    % ノイズレベル (有効/無効の個別制御可能)
    params.noise.enable.accel = true;
    params.noise.enable.gyro = true;
    params.noise.enable.gps = true;
    params.noise.base.accel_std = single(0.1);
    params.noise.base.gyro_std = single(0.5);
end
```

---

## 🔍 デバッグ・診断機能

### 型整合性チェック

```matlab
% 自動型検証 (build_mex.m 内)
if ~isa(obs.ax, 'single')
    error('加速度データは single 型である必要があります');
end
if ~isa(obs.lat, 'double')  
    error('GPS座標は double 型である必要があります');
end
```

### パフォーマンス監視

```matlab
% 実行時間計測 (run_simulation.m 内)
tic;
for k = 1:n_samples
    mex_run_eskf('step', handle, obs, k);
end
execution_time = toc;
fprintf('実行時間: %.2f秒 (%.1f倍速)\n', execution_time, params.T/execution_time);
```

### エラーハンドリング

```matlab
% MEXエラーの安全な処理
try
    handle = mex_run_eskf('init', obs, static_time, dt);
    % ... フィルタ処理 ...
catch ME
    if exist('handle', 'var')
        mex_run_eskf('free', handle);  % リソース確実解放
    end
    rethrow(ME);
end
```

---

## 🎯 品質保証・テスト

### 自動品質チェック

```matlab
% run_batch_10sets.m での自動判定
function status = check_quality_metrics(results)
    pos_rmse = calculate_position_rmse(results);
    att_rmse = calculate_attitude_rmse(results);
    
    status.pos_ok = pos_rmse < 2.0;     % 2m以下
    status.att_ok = att_rmse < 1.0;     % 1度以下  
    status.bias_ok = any(abs(results.bg(:,end)) > 0.01); % バイアス更新確認
    
    status.overall = status.pos_ok && status.att_ok && status.bias_ok;
end
```

### 回帰テストフレームワーク

```matlab
% 過去結果との比較 (Results/reference/ との差分チェック)
function compare_with_reference(current_results)
    ref_file = 'Results/reference/estimation_baseline.csv';
    if exist(ref_file, 'file')
        ref_data = readtable(ref_file);
        current_data = readtable('Results/estimation_01.csv');
        
        rmse_diff = calculate_rmse_difference(ref_data, current_data);
        if rmse_diff > 0.1  % 10cm以上の劣化
            warning('性能劣化検知: RMSE差 %.2fm', rmse_diff);
        end
    end
end
```