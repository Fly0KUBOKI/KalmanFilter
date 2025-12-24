# ESKF MEX移行計画 - 詳細実装ガイド

**補足ドキュメント**: 段階的MEX化の具体的な実装手順と注意点

---

## 1. 関数呼び出し依存グラフ（詳細版）

### 完全な関数相互参照マップ

```
【メインループ】
run_simulation()
  └─ for k=1:N
      └─ update_filter(obs, k)
          ├─ predict(a_meas, w_meas)                    [MATLAB実装]
          │   ├─ Gyro/Accel フィルタ適用
          │   ├─ Adaptive Q計算
          │   └─ obj.call_meukf_step()                 [薄い・MEX呼び出し]
          │       └─ mex_meukf_step_v2(state, sensor_data, params)
          │           ├─ UKF予測
          │           └─ 全センサー更新
          │
          ├─ sensor_updates('accel', a)               [分岐のみ]
          │   └─ update_sensor_impl('accel', a)       [MATLAB実装]
          │       ├─ 変更検知 (norm(a - prev_a) > tol)
          │       ├─ MEX異常検知: mex_sensor_filter('accel', a, ...)
          │       ├─ ノーム・閾値チェック
          │       └─ do_cpp_update('accel', a_corrected)
          │           └─ mex_meukf_step_v2(state, sensor_update_accel, params)
          │
          ├─ sensor_updates('mag', m)
          │   └─ update_sensor_impl('mag', m)
          │       ├─ 変更検知
          │       ├─ mex_sensor_filter('mag', m, prev_m)
          │       └─ do_cpp_update('mag', m_filtered)
          │
          ├─ sensor_updates('gps', lat, lon, alt, k)
          │   └─ update_sensor_impl('gps', ...)
          │       ├─ 変更検知
          │       ├─ GPS → LLA座標変換 [MATLAB]
          │       │   └─ z_gps = [y_m; x_m; -z_m]
          │       └─ do_cpp_update('gps', z_gps)
          │
          ├─ sensor_updates('baro', pressure)
          │   └─ update_sensor_impl('baro', pressure)
          │       ├─ 変更検知
          │       ├─ mex_sensor_filter('baro', pressure)
          │       │   └─ 気圧 → 高度換算
          │       └─ do_cpp_update('baro', alt_baro)
          │
          └─ reset('check', obs, k)
              └─ check_and_reset_impl(obs, k)         [MATLAB実装]
                  ├─ NaN/Inf チェック
                  ├─ 発散判定 (if needed)
                  └─ reset_filter_impl(obs, k)
                      ├─ 共分散リセット
                      └─ GPS統合時: 位置再計算
```

### 階層別依存度

```
Layer 1: 基盤 (Eigen不使用、独立)
├─ get_field_impl()           ← index検索・型変換
├─ has_field_impl()           ← boolean判定
├─ get_euler_impl()           ← mex_quaternion_lib呼び出し [MEX]
└─ delete()                   ← mex_eskf_free呼び出し [MEX]

Layer 2: ユーティリティ (Layer 1を依存)
├─ divergence_check_velocity_impl()  ← 物理制約・行列操作
├─ estimate_noise()                 ← MEX呼び出し or MATLAB calc
├─ get_sensor_R()                   ← MEX or MATLAB取得
├─ reset_sensor_filters()           ← MEX/MATLAB フィルタ管理
└─ [未実装] コンストラクタ初期化完全MEX化

Layer 3: 中レベル (Layer 1-2を依存)
├─ update_sensor_impl()            ← 変更検知＋異常判定＋前処理 [MATLAB厚い]
├─ do_cpp_update()                 ← パラメータ整形＋MEX呼び出し [MATLAB中程度]
├─ divergence_check()              ← MEX分岐
├─ check_and_reset_impl()          ← リセット＋GPS統合 [MATLAB厚い]
├─ reset_filter_impl()             ← 共分散初期化
├─ zupt('check')                   ← 静止判定 [MATLAB薄い]
└─ zupt('update')                  ← ZUPT更新

Layer 4: ロジック (Layer 1-3を依存)
├─ predict()                       ← IMU積分＋Q適応 [MATLAB厚い・CORE]
├─ update_filter()                 ← メインループ分岐 [MATLAB薄い]
├─ sensor_updates()                ← センサー分岐 [MATLAB薄い]
└─ reset()                         ← リセット分岐 [MATLAB薄い]

Layer 5: 外部インタフェース
└─ run_simulation()                ← テスト・検証スクリプト
```

### 数値精度への影響度 (高→低)

```
【高影響】= float精度誤差が最終推定値に大きく影響
1. predict()                     [状態積分・キュムレーティブ]
2. update_sensor_impl()          [センサー変換・前処理]
3. do_cpp_update()               [行列計算・ゲイン]
4. check_and_reset_impl()        [GPS統合・再初期化]

【中影響】= 複合的な影響
5. divergence_check_velocity_impl()  [制約・分散制限]
6. zupt()系                          [補助的更新]

【低影響】= 1回実行・参照的
7. get_field_impl()
8. has_field_impl()
9. get_euler_impl()
10. reset_filter_impl()
```

---

## 2. 各フェーズの具体的実装手順

### Phase 1: 基盤レイヤー - `mex_matlab_helpers.cpp`

**ファイル構成**:

```
kalman/cpp/MEX/
├─ mex_matlab_helpers.cpp          [NEW]
└─ mex_matlab_helpers.mexw64       [OUTPUT]
```

**実装コード例**:

```cpp
// MEX/mex_matlab_helpers.cpp
#include <mex.h>
#include <cstring>
#include <vector>
#include "../include/Common/Math/fixed_matrix.hpp"
#include "../include/Common/Math/quaternion.hpp"

using namespace cmath_fx;

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) {
        mexErrMsgIdAndTxt("mex_matlab_helpers:input", 
            "Usage: result = mex_matlab_helpers(command, ...)");
    }
    
    // Parse command
    char cmd[256];
    if (!mxIsChar(prhs[0])) {
        mexErrMsgIdAndTxt("mex_matlab_helpers:cmd", "Command must be string");
    }
    mxGetString(prhs[0], cmd, 256);
    
    if (strcmp(cmd, "get_field") == 0) {
        // Usage: data = mex_matlab_helpers('get_field', obs_struct, field_names, indices, num_cols)
        if (nrhs < 5) {
            mexErrMsgIdAndTxt("mex_matlab_helpers:get_field", 
                "Usage: data = mex_matlab_helpers('get_field', obs, fields, idx, ncols)");
        }
        
        const mxArray* obs = prhs[1];
        const mxArray* fields_cell = prhs[2];
        double idx_val = mxGetScalar(prhs[3]);
        double ncols_val = mxGetScalar(prhs[4]);
        
        int idx = (int)idx_val - 1;  // MATLAB 1-indexed → C++ 0-indexed
        int ncols = (int)ncols_val;
        
        // Try each field name
        int n_fields = mxGetNumberOfElements(fields_cell);
        bool found = false;
        
        for (int f = 0; f < n_fields; ++f) {
            mxArray* field_mxname = mxGetCell(fields_cell, f);
            if (!mxIsChar(field_mxname)) continue;
            
            char field_name[256];
            mxGetString(field_mxname, field_name, 256);
            
            if (mxIsField(obs, field_name)) {
                mxArray* field_data = mxGetField(obs, 0, field_name);
                
                if (ncols == 1) {
                    // Single column
                    if (mxIsDouble(field_data)) {
                        double* data = mxGetDoubles(field_data);
                        plhs[0] = mxCreateDoubleScalar(data[idx]);
                        return;
                    }
                } else if (ncols == 3) {
                    // 3D vector (might be stored as FieldX, FieldY, FieldZ)
                    char base_name[256] = "";
                    strcpy(base_name, field_name);
                    
                    // Remove suffix if present
                    int len = strlen(base_name);
                    if (len > 0 && (base_name[len-1] == 'x' || 
                                    base_name[len-1] == 'y' || 
                                    base_name[len-1] == 'z')) {
                        base_name[len-1] = '\0';
                    }
                    
                    // Build field names: FieldX, FieldY, FieldZ or Field_x, Field_y, Field_z
                    char suffix[5] = "_xyz";
                    double* vx = nullptr, * vy = nullptr, * vz = nullptr;
                    
                    // Try FieldX format
                    char field_x[256], field_y[256], field_z[256];
                    snprintf(field_x, 256, "%sx", base_name);
                    snprintf(field_y, 256, "%sy", base_name);
                    snprintf(field_z, 256, "%sz", base_name);
                    
                    if (mxIsField(obs, field_x)) {
                        vx = mxGetDoubles(mxGetField(obs, 0, field_x));
                        vy = mxGetDoubles(mxGetField(obs, 0, field_y));
                        vz = mxGetDoubles(mxGetField(obs, 0, field_z));
                    } else {
                        // Try Field_x format
                        snprintf(field_x, 256, "%s_x", base_name);
                        snprintf(field_y, 256, "%s_y", base_name);
                        snprintf(field_z, 256, "%s_z", base_name);
                        if (mxIsField(obs, field_x)) {
                            vx = mxGetDoubles(mxGetField(obs, 0, field_x));
                            vy = mxGetDoubles(mxGetField(obs, 0, field_y));
                            vz = mxGetDoubles(mxGetField(obs, 0, field_z));
                        }
                    }
                    
                    if (vx && vy && vz) {
                        plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL);
                        double* out = mxGetDoubles(plhs[0]);
                        out[0] = vx[idx];
                        out[1] = vy[idx];
                        out[2] = vz[idx];
                        return;
                    }
                }
            }
        }
        
        mexErrMsgIdAndTxt("mex_matlab_helpers:field_not_found", 
            "None of the specified fields found in struct");
            
    } else if (strcmp(cmd, "has_field") == 0) {
        // Usage: has_it = mex_matlab_helpers('has_field', obs_struct, field_names)
        if (nrhs < 3) {
            mexErrMsgIdAndTxt("mex_matlab_helpers:has_field", 
                "Usage: has_it = mex_matlab_helpers('has_field', obs, fields)");
        }
        
        const mxArray* obs = prhs[1];
        const mxArray* fields_cell = prhs[2];
        int n_fields = mxGetNumberOfElements(fields_cell);
        
        for (int f = 0; f < n_fields; ++f) {
            mxArray* field_mxname = mxGetCell(fields_cell, f);
            if (!mxIsChar(field_mxname)) continue;
            
            char field_name[256];
            mxGetString(field_mxname, field_name, 256);
            
            if (mxIsField(obs, field_name)) {
                plhs[0] = mxCreateLogicalScalar(true);
                return;
            }
        }
        
        plhs[0] = mxCreateLogicalScalar(false);
        
    } else if (strcmp(cmd, "get_euler") == 0) {
        // Usage: euler_deg = mex_matlab_helpers('get_euler', quaternion)
        // Delegates to mex_quaternion_lib internally or re-implements
        if (nrhs < 2) {
            mexErrMsgIdAndTxt("mex_matlab_helpers:get_euler", 
                "Usage: euler = mex_matlab_helpers('get_euler', quat)");
        }
        
        double* q_data = mxGetDoubles(prhs[1]);
        Vector4 q;
        q(0, 0) = (float)q_data[0];  // w
        q(1, 0) = (float)q_data[1];  // x
        q(2, 0) = (float)q_data[2];  // y
        q(3, 0) = (float)q_data[3];  // z
        
        // Call quaternion library
        Vector3 euler = cquat::quat_to_euler(q);  // in radians
        
        // Convert to degrees
        Vector3 euler_deg;
        euler_deg(0, 0) = euler(0, 0) * 180.0f / 3.14159265f;
        euler_deg(1, 0) = euler(1, 0) * 180.0f / 3.14159265f;
        euler_deg(2, 0) = euler(2, 0) * 180.0f / 3.14159265f;
        
        plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL);
        double* out = mxGetDoubles(plhs[0]);
        out[0] = (double)euler_deg(0, 0);
        out[1] = (double)euler_deg(1, 0);
        out[2] = (double)euler_deg(2, 0);
        
    } else {
        mexErrMsgIdAndTxt("mex_matlab_helpers:unknown_cmd", 
            "Unknown command: %s", cmd);
    }
}
```

**MATLAB側の置き換え**:

```matlab
% BEFORE:
function data = get_field_impl(obj, obs, field_names, idx, num_cols)
    for i = 1:length(field_names)
        if isfield(obs, field_names{i})
            % ...複雑なロジック...
        end
    end
end

% AFTER:
function data = get_field_impl(obj, obs, field_names, idx, num_cols)
    if exist('mex_matlab_helpers', 'file') == 3
        data = mex_matlab_helpers('get_field', obs, field_names, idx, num_cols);
    else
        % MATLAB フォールバック
        for i = 1:length(field_names)
            % ...元のロジック...
        end
    end
end
```

**ビルド手順**:

```matlab
cd kalman/cpp/build
build_mex({'mex_matlab_helpers'})
clear mex
cd ../..
```

**検証テスト**:

```matlab
% Test get_field_impl
obs.accel_x = [1, 2, 3];
obs.accel_y = [4, 5, 6];
obs.accel_z = [7, 8, 9];

result = mex_matlab_helpers('get_field', obs, {'accel_x'}, 2, 3);
expected = [2; 5; 8];
assert(norm(result - expected) < 1e-10);

% Test has_field
has_accel = mex_matlab_helpers('has_field', obs, {'accel_x', 'mag_x'});
assert(has_accel == true);

% Test get_euler
quat = [1; 0; 0; 0];  % Identity
euler = mex_matlab_helpers('get_euler', quat);
assert(norm(euler) < 1e-5);

fprintf('Phase 1: All tests passed!\n');
```

---

### Phase 3: センサー前処理 - 具体的な実装チェックポイント

**変更検知ロジックのC++化**:

```cpp
// src/ESKF/sensor_preprocessor.hpp
struct SensorPreprocessOutput {
    bool is_outlier;
    Vector3 corrected;  // For vector sensors
    float confidence;
};

class SensorPreprocessor {
public:
    static SensorPreprocessOutput preprocess_accel(
        const Vector3& a_meas,
        const Vector3& prev_a,
        float buffer_tolerance = 1e-9f,
        float gravity = 9.81f,
        float max_deviation = 3.0f
    ) {
        SensorPreprocessOutput out;
        out.corrected = a_meas;
        out.is_outlier = false;
        out.confidence = 1.0f;
        
        // 変更検知
        Vector3 delta = a_meas - prev_a;
        float delta_norm = 0.0f;
        for (int i = 0; i < 3; ++i) delta_norm += delta(i, 0) * delta(i, 0);
        delta_norm = std::sqrt(delta_norm);
        
        if (delta_norm <= buffer_tolerance) {
            return out;  // No change, reuse previous
        }
        
        // ノーム・範囲チェック
        float a_norm = 0.0f;
        for (int i = 0; i < 3; ++i) a_norm += a_meas(i, 0) * a_meas(i, 0);
        a_norm = std::sqrt(a_norm);
        
        if (a_norm < 0.1f || std::abs(a_norm - gravity) > max_deviation) {
            out.is_outlier = true;
            out.confidence = 0.0f;
            return out;
        }
        
        out.confidence = 1.0f / (1.0f + std::abs(a_norm - gravity) / gravity);
        return out;
    }
};
```

**GPS座標変換のC++化**:

```cpp
static SensorPreprocessOutput preprocess_gps(
    float lat, float lon, float alt,
    const Vector3& origin,
    float buffer_tolerance = 1e-9f
) {
    SensorPreprocessOutput out;
    out.is_outlier = false;
    out.confidence = 1.0f;
    
    // 変更検知
    float dlat = lat - origin(0, 0);
    float dlon = lon - origin(1, 0);
    float dalt = alt - origin(2, 0);
    
    if (std::abs(dlat) <= buffer_tolerance && 
        std::abs(dlon) <= buffer_tolerance && 
        std::abs(dalt) <= buffer_tolerance) {
        out.corrected = Vector3{0.0f, 0.0f, 0.0f};
        return out;  // No change
    }
    
    // LLA → ENU変換 (MATLAB実装を移行)
    float lat0 = origin(0, 0);
    float lon0 = origin(1, 0);
    
    float y_m = dlat / 9.0e-6f;
    float x_m = dlon / (9.0e-6f / std::cos(lat0 * 3.14159265f / 180.0f));
    float z_m = -dalt;
    
    out.corrected(0, 0) = y_m;
    out.corrected(1, 0) = x_m;
    out.corrected(2, 0) = z_m;
    
    return out;
}
```

---

## 3. 数値精度の注意点

### 3.1 float vs double の混在リスク

**既存問題** (TYPE_MIX_REPORT.md):
- MATLAB: double (64-bit)
- MEX: float (32-bit)

**Phase別対策**:

| Phase | 対策 |
|-------|------|
| 1-2 | MEX入力を `(float)` キャストして変換 |
| 3-4 | 共分散の対称化を必須化 (`P = (P+P')/2`) |
| 5+ | 各段階で `run_batch_10sets()` で検証 |

### 3.2 クォータニオン正規化

**必須箇所**:
```cpp
// 四則演算後
q = q / q.norm();  // または cquat::normalize_quat(q);

// 出力前
float q_norm = std::sqrt(q(0,0)*q(0,0) + q(1,0)*q(1,0) + q(2,0)*q(2,0) + q(3,0)*q(3,0));
if (std::abs(q_norm - 1.0f) > 1e-5f) {
    q = q / q_norm;
}
```

### 3.3 行列対称化の必須化

**出力前**:
```cpp
// Phase 3以降全てのフェーズで
void symmetrize_matrix(Matrix15x15& P) {
    for (int i = 0; i < 15; ++i) {
        for (int j = i+1; j < 15; ++j) {
            float val = (P(i, j) + P(j, i)) * 0.5f;
            P(i, j) = val;
            P(j, i) = val;
        }
    }
}
```

---

## 4. テスト検証フロー

### 4.1 各フェーズのテストチェックリスト

**Phase 1テスト**:
```matlab
% 1. ビルド確認
cd kalman/cpp/build
build_mex({'mex_matlab_helpers'})

% 2. 基本動作テスト
clear mex
obs.accel_x = rand(100, 1);
obs.accel_y = rand(100, 1);
obs.accel_z = rand(100, 1);
data = mex_matlab_helpers('get_field', obs, {'accel_x'}, 50, 3);
assert(size(data, 1) == 3);

% 3. 実シミュレーション
run_simulation(42, true)
% 確認: Results/estimation_0.csv が生成されること

% 4. 比較テスト
compare_mex_matlab_detailed()
% 確認: 数値差分が1e-12以下（同一実装）
```

**Phase 3テスト**:
```matlab
% 前提: Phase 1-2完了
build_mex({'mex_sensor_preprocessor'})
clear mex

% 1. 基本動作テスト
a_meas = [9.8; 0.1; 0.05];
a_prev = [9.8; 0.1; 0.05];
[is_outlier, corrected] = mex_sensor_preprocessor('preprocess_accel', a_meas, a_prev);
assert(is_outlier == false);

% 2. GPS変換テスト
origin = [35.0; 139.0; 0.0];
[z_gps, is_outlier] = mex_sensor_preprocessor('preprocess_gps', ...
    35.0001, 139.0001, 10.0, origin);

% 3. 全ステップテスト
run_simulation(42, true)

% 4. 精度検証
compare_mex_matlab_detailed()
% 許容: RMS <1e-4（異なる実装なのでfloat精度誤差許容）

% 5. バッチ回帰テスト
run_batch_10sets()
% 確認: 全セット正常終了、RMS Error が前フェーズと±5%以内
```

**Phase 4テスト** (最重要):
```matlab
% 前提: Phase 1-3完了
build_mex({'mex_adaptive_predict'})  % または mex_meukf_step_v2 拡張
clear mex

% 1. 予測ステップ単体テスト
state.p = [0; 0; 0];
state.v = [0; 0; 0];
state.q = [1; 0; 0; 0];
state.ba = [0; 0; 0];
state.bg = [0; 0; 0];
state.P = eye(15) * 0.01;

a_meas = [0; 0; 9.8];
w_meas = [0; 0; 0];
dt = 0.01;

[state_new] = mex_adaptive_predict('predict', state, a_meas, w_meas, dt, ...);
assert(~any(isnan([state_new.p; state_new.v])));

% 2. 完全シミュレーション
run_simulation(42, true)

% 3. 詳細比較 (Phase 4は最高精度テストが必須)
[results_matlab, results_mex] = compare_mex_matlab_detailed();
% チェック:
%  - position RMS < 1e-3 m
%  - velocity RMS < 1e-4 m/s
%  - attitude RMS < 0.1 deg
%  - NaN/Inf なし

% 4. バッチテスト
run_batch_10sets()
% すべてのセットが成功
% RMS値が Phase 3 ±3% 以内

% 5. パフォーマンス
tic; for iter=1:10, run_simulation(iter, false); end; t1=toc;
fprintf('Time for 10 simulations (Phase 4): %.2f s\n', t1);
% 期待: Phase 3より 10-20% 高速化
```

---

## 5. ビルド設定の更新

### 5.1 build_mex.m への追加

```matlab
% In build_mex.m, add to MEX file list:

% Phase 1
targets_config = {
    % ... existing targets ...
    
    % Phase 1: Helpers
    struct('name', 'mex_matlab_helpers', 'src', 'mex_matlab_helpers.cpp', ...
           'deps', {}),
    
    % Phase 2: (already exists - mex_sensor_filter)
    
    % Phase 3: Sensor preprocessor
    struct('name', 'mex_sensor_preprocessor', 'src', 'mex_sensor_preprocessor.cpp', ...
           'deps', {'../src/ESKF/sensor_preprocessor.cpp'}),
    
    % Phase 4: Adaptive predict
    struct('name', 'mex_adaptive_predict', 'src', 'mex_adaptive_predict.cpp', ...
           'deps', {'../src/ESKF/adaptive_predict.cpp'}),
    
    % Phase 5: Filter management
    struct('name', 'mex_filter_management', 'src', 'mex_filter_management.cpp', ...
           'deps', {'../src/ESKF/filter_management.cpp'}),
};
```

---

## 6. エラーハンドリング・デバッグ

### 6.1 よくあるビルドエラーと対処

| エラー | 原因 | 対処 |
|--------|------|------|
| `undefined reference to cquat::...` | ヘッダインクルード漏れ | `#include "../include/Common/Math/quaternion.hpp"` |
| `size mismatch in Matrix assignment` | テンプレート型不一致 | `Vector3` と `Vector<3, float>` を統一 |
| `mxGetDoubles not found` | MATLAB古いバージョン | `mxGetPr` を使用 |
| `NaN in output` | float精度誤差 | 共分散対称化・正規化を確認 |

### 6.2 デバッグ出力の有効化

```cpp
// MEX内でのデバッグプリント
#ifdef DEBUG_PHASE_X
    mexPrintf("[DEBUG] sensor=%s, outlier=%d, confidence=%f\n", 
              sensor_name, is_outlier, confidence);
#endif
```

**コンパイル時フラグ**:
```matlab
% In build_mex.m
compile_opts = [compile_opts, '-DDEBUG_PHASE_3'];  % Phase 3デバッグ有効化
```

---

## 7. フェーズ間の互換性チェック

### 7.1 状態ベクトル一貫性

各フェーズで状態ベクトルの順序を確認:
```
✅ [p(3), v(3), q(4), ba(3), bg(3)] = 15要素 固定
✅ q = [w, x, y, z] スカラー先頭
✅ 出力前に共分散対称化
```

### 7.2 MEX出力の検証

```matlab
function validate_mex_output(state)
    % 状態チェック
    assert(length(state.p) == 3, 'Position size mismatch');
    assert(length(state.q) == 4, 'Quaternion size mismatch');
    assert(state.q(1)^2 + state.q(2)^2 + state.q(3)^2 + state.q(4)^2 < 1.01, 'Quaternion norm > 1.01');
    assert(~any(isnan(state.p)), 'NaN in position');
    assert(~any(isinf(state.P(:))), 'Inf in covariance');
    
    % 共分散チェック
    eigvals = eig(state.P);
    assert(all(eigvals > -1e-6), 'Negative eigenvalue in covariance');
end
```

---

## 8. ロールバック手順

**問題発生時**:

```matlab
% 1. MEXバイナリ削除
delete('kalman/cpp/bin/mex_新規MEX名.mexw64');

% 2. MEXビルド設定から削除
% → build_mex.m のターゲットリストから該当行を削除

% 3. MATLAB側を復旧
% → ESKF.m の関数を元のMATLAB実装に戻す

% 4. テスト
clear mex
run_simulation(42, true)
run_batch_10sets()

% 5. git commit（破棄）
git reset --hard HEAD~1
```

---

**次ステップ**: Phase 1のmex_matlab_helpers.cppのビルド実行

