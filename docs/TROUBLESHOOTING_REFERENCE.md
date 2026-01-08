# トラブルシューティング・技術資料

## 🚨 既知の問題と解決法

### Innovation計算問題 (現在進行中)

**症状**: Max Innovation が常に 0.0000 を示す
- **影響**: センサー更新の診断ができない（フィルタ自体は正常動作）
- **原因**: `handle_sensor_update_internal()` でのInnovation返却処理未完成
- **対処**: アルゴリズムは正常に動作しているため、診断情報の実装を待つ

**調査記録**:
```
現在の成果 (2026/1/4):
- Position RMSE: 0.80-0.91m (目標達成)
- Attitude RMSE: 0.25-0.30° (目標達成) 
- 成功率: 100% (10/10) (目標達成)
- Gyro bias更新: 正常 (非ゼロ値収束)

残る課題:
- Innovation診断情報の可視化のみ
```

---

## 📋 MEX入出力仕様 (完全版)

### データ型マッピング

| 方向 | MATLAB型 | C++型 | 変換関数 | 用途 |
|-----|---------|--------|---------|------|
| In | `single` 3×1 | `float[3]` | `mxArrayToFloatArray()` | 加速度・ジャイロ・磁気 |
| In | `double` 3×1 | `double[3]` | `mxGetPr()` | GPS座標 (高精度) |
| In | `logical` | `uint8_t` | `mxIsLogicalScalarTrue()` | 更新フラグ |
| Out | `float[3]` | `single` 3×1 | `mxCreateNumericMatrix()` | 位置・速度・バイアス |
| Out | `float[4]` | `single` 4×1 | `mxCreateNumericMatrix()` | 四元数 [w,x,y,z] |
| Out | `float[15×15]` | `single` 15×15 | `mxCreateNumericMatrix()` | 共分散 (column-major) |

### MEX API呼び出し仕様

```matlab
% 初期化
handle = mex_run_eskf('init', obs, static_time, dt);
% obs: 観測構造体 (ax,ay,az,wx,wy,wz,mx,my,mz,pressure,lat,lon,alt)
% static_time: 初期化期間 [秒] (通常5.0)
% dt: サンプリング間隔 [秒] (通常0.0025)
% 戻り値: ハンドル (uint64)

% ステップ実行
mex_run_eskf('step', handle, obs, k);
% handle: 初期化で取得
% obs: 現タイムステップ観測値
% k: ステップ番号 (1-based)

% 状態取得
state = mex_run_eskf('get_state', handle);
% 戻り値: 構造体
%   .p: 位置 [m]
%   .v: 速度 [m/s]
%   .q: 四元数 [w,x,y,z] (正規化済み)
%   .euler: オイラー角 [deg] (Roll/Pitch/Yaw)
%   .ba: 加速度バイアス [m/s²]
%   .bg: ジャイロバイアス [rad/s] 
%   .P: 共分散行列 15×15 (対称化済み)

% 解放
mex_run_eskf('free', handle);
```

---

## ⚡ 最適化・パフォーマンス

### 型変換の最適化原則

**GPS以外はfloat32で統一**:
```matlab
% 正しい型設定
obs.ax = single(raw_data.accel_x);  % センサーデータは single
obs.lat = double(raw_data.gps_lat); % GPS座標は double

% 間違った型設定 (実行時エラーになる)
obs.ax = double(raw_data.accel_x);  % MEX側でエラー
obs.lat = single(raw_data.gps_lat); % 精度劣化
```

### メモリ効率化

**固定サイズ行列の使用**:
```cpp
// C++側: 動的確保を避ける
Matrix<15,15,float> P_;  // スタック確保
Vector<15,float> state_; // 固定サイズベクトル

// 避けるべき: 動的確保
Eigen::MatrixXf P_;      // ヒープ確保（遅い）
```

**MEXハンドル管理**:
```cpp
// 安全なハンドル管理
static std::map<uint64_t, ESKFState*> g_states;
static uint64_t g_next_handle = 1;

// メモリリーク防止
~ESKFState() {
    // 自動クリーンアップ
}
```

---

## 🔬 数値安定性・品質管理

### 四元数正規化の厳格化

**常に正規化を確認**:
```cpp
void normalize_quaternion(float q[4]) {
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 1e-8f) {  // ゼロ除算防止
        q[0] /= norm; q[1] /= norm; q[2] /= norm; q[3] /= norm;
    }
    
    // 検証アサーション (DEBUG時)
    #ifdef DEBUG
    float check = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    assert(fabsf(check - 1.0f) < 1e-6f);
    #endif
}
```

### 共分散対称化の強制

**数値誤差による非対称性を防ぐ**:
```cpp
void symmetrize_covariance(float P[15*15]) {
    for (int i = 0; i < 15; ++i) {
        for (int j = i + 1; j < 15; ++j) {
            float avg = 0.5f * (P[i*15 + j] + P[j*15 + i]);
            P[i*15 + j] = avg;
            P[j*15 + i] = avg;
        }
    }
}
```

### 品質保証メトリクス

**自動品質チェック基準**:
```matlab
% 必達条件
pos_rmse_max = 2.0;      % Position RMSE < 2.0m
att_rmse_max = 1.0;      % Attitude RMSE < 1.0°
success_rate_min = 90.0; % 成功率 > 90% (10seed中)

% 警告条件  
pos_rmse_warning = 1.5;  % Position RMSE > 1.5m で警告
att_rmse_warning = 0.5;  % Attitude RMSE > 0.5° で警告

% バイアス収束確認
gyro_bias_min = 0.01;    % |bg_final| > 0.01 (rad/s) で正常収束
```

---

## 🛠️ デバッグ・診断技法

### 段階的デバッグ手順

**Phase 1: 基本動作確認**
```matlab
% MEXロード確認
which mex_run_eskf  % MEXファイル位置確認

% 型チェック
obs = load_sensor_data();
assert(isa(obs.ax, 'single'), 'Accel type mismatch');
assert(isa(obs.lat, 'double'), 'GPS type mismatch');
```

**Phase 2: 初期化検証**
```matlab 
handle = mex_run_eskf('init', obs, 5.0, 0.0025);
state = mex_run_eskf('get_state', handle);

% 初期状態確認
assert(all(abs(state.p) < 1e-3), 'Initial position not zero');
assert(abs(norm(state.q) - 1.0) < 1e-6, 'Initial quaternion not normalized');
```

**Phase 3: ステップ実行検証**
```matlab
for k = 1:100  % 最初の100ステップ
    mex_run_eskf('step', handle, obs, k);
    state = mex_run_eskf('get_state', handle);
    
    % 異常値検出
    if any(isnan([state.p; state.v; state.q]))
        error('NaN detected at step %d', k);
    end
    
    if norm(state.q) < 0.9 || norm(state.q) > 1.1
        error('Quaternion denormalized at step %d: norm=%.6f', k, norm(state.q));
    end
end
```

### 高度な診断機能

**センサー統計診断**:
```matlab
function diagnose_sensor_quality(obs)
    % データ統計
    fprintf('=== センサー品質診断 ===\n');
    fprintf('加速度: mean=%.3f, std=%.3f, range=[%.2f, %.2f]\n', ...
        mean(obs.ax), std(obs.ax), min(obs.ax), max(obs.ax));
    fprintf('ジャイロ: mean=%.3f, std=%.3f, range=[%.2f, %.2f]\n', ...
        mean(obs.wx), std(obs.wx), min(obs.wx), max(obs.wx));
    
    % 外れ値カウント
    accel_outliers = sum(abs(obs.ax - mean(obs.ax)) > 3*std(obs.ax));
    gyro_outliers = sum(abs(obs.wx - mean(obs.wx)) > 3*std(obs.wx));
    
    fprintf('外れ値検出: 加速度=%d個, ジャイロ=%d個\n', accel_outliers, gyro_outliers);
    
    % データ完整性
    assert(~any(isnan(obs.ax)), '加速度データにNaN');
    assert(~any(isnan(obs.lat)), 'GPS座標にNaN');
    assert(length(obs.ax) == length(obs.lat), 'データ長不整合');
    
    fprintf('データ完整性: OK\n');
end
```

**フィルタ収束診断**:
```matlab
function diagnose_filter_convergence(results)
    % バイアス収束確認
    bg_norm_final = norm(results.bg(:, end));
    if bg_norm_final < 0.01
        warning('ジャイロバイアス未収束: final norm = %.6f rad/s', bg_norm_final);
    else
        fprintf('ジャイロバイアス正常収束: %.4f rad/s\n', bg_norm_final);
    end
    
    % 共分散収束確認
    P_diag_final = diag(squeeze(results.P(:,:,end)));
    pos_uncertainty = sqrt(sum(P_diag_final(1:3)));  % Position uncertainty
    att_uncertainty = sqrt(sum(P_diag_final(7:9)));  % Attitude uncertainty
    
    fprintf('最終不確実性: 位置=%.3fm, 姿勢=%.3f°\n', pos_uncertainty, att_uncertainty*180/pi);
    
    % 発散チェック
    if pos_uncertainty > 10.0
        error('位置不確実性が発散: %.1fm', pos_uncertainty);
    end
end
```

---

## 📚 アーキテクチャ補足情報

### ライブラリ依存関係

```
mex_run_eskf.mexw64
├── ESKF Core
│   ├── eskf_core.cpp (主フィルタ実装)
│   ├── eskf_sensor_updates.cpp (センサー統合)
│   ├── eskf_math.cpp (数学関数)
│   └── eskf_initializer.cpp (初期化)
├── Common Libraries  
│   ├── Math/math_utils.hpp (統計・ロバスト推定)
│   ├── Sensor/sensor_filter.hpp (外れ値検出)
│   └── filter_mgmt.cpp (状態管理)
├── Matrix Operations
│   └── fixed_matrix.hpp (15x15特化演算)
└── Quaternion Functions
    └── quaternion_functions.hpp (回転演算)
```

### 名前空間構成

```cpp
namespace kalman {
    namespace eskf {
        class ESKFCore;        // 主フィルタクラス  
        struct ESKFState;      // 状態構造体
        // ESKF専用関数群
    }
    
    namespace common {
        class OutlierDetector; // 外れ値検出
        class SensorFilter;    // センサー前処理
        // 共通ユーティリティ
    }
    
    namespace math {
        struct Quaternion;     // 四元数型
        class Matrix15x15;     // 固定サイズ行列
        // 数学関数群
    }
}
```

---

## 🎯 将来拡張・ロードマップ

### Phase 13 計画 (進行中)

1. **Innovation計算の完全実装**
   - `last_innov_norm`, `last_maha_dist` の正確な値返却
   - センサー更新診断の可視化

2. **スタンドアロンC++版の提供**
   - MATLAB非依存の実行可能ファイル
   - CMakeベースビルドシステム

3. **追加センサー対応**
   - カメラ (Visual-Inertial SLAM)
   - LiDAR (3D点群統合)
   - UWB (Ultra-Wideband測距)

### 技術的改善案

**パフォーマンス向上**:
- SIMD命令による行列演算高速化
- GPU演算の活用 (CUDA/OpenCL)
- マルチスレッド化 (センサー並列処理)

**アルゴリズム強化**:
- Adaptive R行列 (動的ノイズ推定)
- 非線形観測モデル (高精度GPS)
- 制約付きKalmanフィルタ (物理制約活用)