# C++関数入出力仕様

## データ型マッピング

### MATLAB → C++

| MATLAB型 | C++型 | 変換関数 | 備考 |
|---------|-------|---------|------|
| `single` (3x1) | `float[3]` | `mxArrayToFloatArray()` | GPS以外のセンサーデータ |
| `single` (4x1) | `float[4]` | `mxArrayToFloatArray()` | クォータニオン |
| `single` (15x15) | `float[15*15]` | `mxArrayToFloatArray()` | 共分散行列（row-major変換あり） |
| `double` (3x1) | `double[3]` | `mxGetPr()` | GPSデータのみ |
| `double` (scalar) | `double` | `mxGetScalar()` | 時間、パラメータ |
| `logical` (scalar) | `uint8_t` | `mxIsLogicalScalarTrue()` | 更新フラグ |

### C++ → MATLAB

| C++型 | MATLAB型 | 変換方法 | 備考 |
|-------|---------|---------|------|
| `float[3]` | `single` (3x1) | `mxCreateNumericMatrix(3, 1, mxSINGLE_CLASS, mxREAL)` | 位置、速度、バイアス |
| `float[4]` | `single` (4x1) | `mxCreateNumericMatrix(4, 1, mxSINGLE_CLASS, mxREAL)` | クォータニオン |
| `float[15*15]` | `single` (15x15) | `mxCreateNumericMatrix(15, 15, mxSINGLE_CLASS, mxREAL)` | 共分散行列（column-major変換あり） |
| `double[3]` | `double` (3x1) | `mxCreateDoubleMatrix(3, 1, mxREAL)` | GPSデータ（使用されていない可能性） |

## 主要関数の入出力仕様

### 1. mex_run_eskf('init', ...)

#### 入力
```matlab
obs: struct
  - ax, ay, az: single[] (加速度)
  - wx, wy, wz: single[] (ジャイロ)
  - mx, my, mz: single[] (磁気)
  - pressure: single[] (気圧)
  - lat, lon, alt: double[] (GPS)
static_time: double (静止時間 [s])
dt: double (サンプリング時間 [s])
```

#### 出力
```matlab
handle: uint64 (状態ハンドル)
```

---

### 2. mex_run_eskf('step', ...)

#### 入力
```matlab
handle: uint64 (状態ハンドル)
obs: struct
  - ax, ay, az: single[] (加速度)
  - wx, wy, wz: single[] (ジャイロ)
  - mx, my, mz: single[] (磁気)
  - pressure: single[] (気圧)
  - lat, lon, alt: double[] (GPS)
k: double (ステップ番号、1-based)
```

#### 出力
```matlab
なし（状態を内部更新）
```

---

### 3. mex_run_eskf('get_state', ...)

#### 入力
```matlab
handle: uint64 (状態ハンドル)
```

#### 出力
```matlab
state: struct
  - p: single[3] (位置 [m])
  - v: single[3] (速度 [m/s])
  - q: single[4] (クォータニオン [w, x, y, z])
  - euler: single[3] (オイラー角 [deg])
  - ba: single[3] (加速度バイアス [m/s^2])
  - bg: single[3] (ジャイロバイアス [rad/s])
  - P: single[15x15] (共分散行列)
```

---

### 4. mex_run_eskf('meukf_step', ...)

#### 入力
```matlab
prev_state: struct
  - p: single[3] (位置)
  - v: single[3] (速度)
  - q: single[4] (クォータニオン)
  - ba: single[3] (加速度バイアス)
  - bg: single[3] (ジャイロバイアス)
  - P: single[15x15] (共分散行列、column-major)

sensor: struct
  - accel: single[3] (加速度 [m/s^2])
  - gyro: single[3] (ジャイロ [rad/s])
  - mag: single[3] (磁気 [uT])
  - gps_pos: double[3] (GPS位置、LLAまたはNED)
  - alt_baro: single (気圧高度 [m])
  - prev_mag: single[3] (前回磁気値)
  - prev_gps_pos: double[3] (前回GPS位置)
  - prev_baro_alt: single (前回気圧高度)
  - update_accel: logical or single/double (更新フラグ)
  - update_gyro: logical or single/double
  - update_mag: logical or single/double
  - update_gps: logical or single/double
  - update_baro: logical or single/double
  - update_zupt: logical or single/double
  - dt: single or double (時間ステップ [s])

params: struct
  - g: single[3] (重力ベクトル)
  - mag_ref: single[3] (基準磁気ベクトル)
  - noise_accel: single[3] (加速度ノイズ分散)
  - noise_gyro: single[3] (ジャイロノイズ分散)
  - noise_ba: single[3] (加速度バイアスノイズ分散)
  - noise_bg: single[3] (ジャイロバイアスノイズ分散)
  - noise_mag: single[3] (磁気ノイズ分散)
  - noise_gps: double[3] (GPSノイズ分散 [m^2])
  - noise_baro: single (気圧ノイズ分散 [m^2])
  - noise_zupt: single[3] (ZUPTノイズ分散)
  - alpha: single (UKFパラメータ)
  - beta: single (UKFパラメータ)
  - kappa: single (UKFパラメータ)
```

#### 出力
```matlab
new_state: struct (prev_stateと同じ構造、値が更新される)
  - p: single[3]
  - v: single[3]
  - q: single[4]
  - ba: single[3]
  - bg: single[3]
  - P: single[15x15] (column-major)

dbg_out: struct (デバッグ情報)
  - innov: (イノベーション)
  - H: (観測行列)
  - dx: (誤差状態)

dbg_output: struct (詳細デバッグ情報)
  - pred_P: single[15x15] (予測共分散)
  - last_K: single[15x3] (カルマンゲイン)
  - last_S: single[3x3] (イノベーション共分散)
  - last_S_inv: single[3x3] (イノベーション共分散の逆行列)
  - last_H: single[3x15] (観測行列)
  - last_y: single[3] (イノベーションベクトル)
  - last_y_len: double (イノベーション長)
  - last_sensor_type: double (センサータイプ: 0=none, 1=accel, 2=mag, 3=gps, 4=baro)
  - input_update_gps: double (入力GPS更新フラグ)
  - input_noise_gps: single[3] (入力GPSノイズ)
```

---

## 内部データ構造

### ESKFState

```cpp
struct ESKFState {
    // 状態変数 (double型)
    double p[3];           // 位置 [m]
    double v[3];           // 速度 [m/s]
    double q[4];           // クォータニオン
    double ba[3];          // 加速度バイアス [m/s^2]
    double bg[3];          // ジャイロバイアス [rad/s]
    
    // 共分散行列 (double型, column-major)
    double P[15*15];       // 誤差共分散行列
    
    // パラメータ
    double g[3];           // 重力ベクトル
    double gps_origin[3];  // GPS原点
    double dt;             // サンプリング時間 [s]
    double Q_nominal[15*15]; // ノミナルプロセスノイズ
    
    // 前回値（変更検知用）
    double prev_accel[3];
    double prev_gyro[3];
    double prev_mag[3];
    double prev_gps_lat, prev_gps_lon, prev_gps_alt;
    double prev_baro;
    
    // その他
    bool valid;
    bool adaptive_q_enabled;
    bool enable_accel_z_integration;
    // ...
};
```

### MEUKF State

```cpp
struct State {
    float p[3];         // 位置 [m]
    float v[3];         // 速度 [m/s]
    float q[4];         // クォータニオン
    float ba[3];        // 加速度バイアス [m/s^2]
    float bg[3];        // ジャイロバイアス [rad/s]
    float P[15*15];     // 共分散行列 (row-major)
};
```

### MEUKFInput

```cpp
struct MEUKFInput {
    State prev_state;       // 前回状態
    SensorData sensor;      // センサーデータ
    Params params;          // パラメータ
};
```

### MEUKFOutput

```cpp
struct MEUKFOutput {
    State new_state;        // 更新後状態
    float debug_info[10];   // デバッグ情報
    uint8_t status;         // ステータス (0:正常, 1:エラー)
    float last_K[15*3];     // カルマンゲイン (row-major)
    float last_S[3*3];      // イノベーション共分散 (row-major)
    float last_S_inv[3*3];  // イノベーション共分散の逆行列 (row-major)
    float last_H[3*15];     // 観測行列 (row-major)
    float last_y[3];        // イノベーションベクトル
    uint8_t last_y_len;     // イノベーション長
    uint8_t last_sensor_type; // センサータイプ
    float pred_P[15*15];    // 予測共分散 (row-major)
};
```

---

## 行列ストレージ形式

### MATLAB (column-major)
```
A(i,j) → data[j*rows + i]
```

### C++内部 (row-major)
```
A(i,j) → data[i*cols + j]
```

### 変換処理

#### MATLAB → C++ (column-major → row-major)
```cpp
// P: MATLAB column-major → C++ row-major
for (int r=0; r<15; ++r) {
    for (int c=0; c<15; ++c) {
        c_state.P[r*15 + c] = P_tmp[c*15 + r];
    }
}
```

#### C++ → MATLAB (row-major → column-major)
```cpp
// P: C++ row-major → MATLAB column-major
for (int c=0; c<15; ++c) {
    for (int r=0; r<15; ++r) {
        pf[r + c*15] = state.P[r*15 + c];
    }
}
```

---

## 型変換の制約

### GPSデータ
- **入力**: `double`型のみ受け付け
- **出力**: `double`型（ただし、現在の実装では`float`型で出力）
- **理由**: GPS座標の精度要件

### その他のセンサーデータ
- **入力**: `single`型のみ受け付け（型変換を廃止）
- **出力**: `single`型
- **理由**: パフォーマンスと一貫性

### 更新フラグ
- **入力**: `logical`, `single`, `double`のいずれも受け付け
- **処理**: `logical`は`true`→1, `false`→0に変換

---

## エラーハンドリング

### 型エラー
```cpp
mexErrMsgIdAndTxt("mex_run_eskf:type_error", 
    "Expected single (float) array for field '%s', but got %s.", 
    field_name, mxGetClassName(arr));
```

### 無効なハンドル
```cpp
mexErrMsgIdAndTxt("mex_run_eskf:invalid", "Invalid handle");
```

### 無効なパラメータ
```cpp
mexErrMsgIdAndTxt("MEUKF:step:invalidNoiseGPS", 
    "noise_gps must be finite non-negative variances (meters^2).");
```

---

## パフォーマンス考慮事項

1. **型変換の最小化**: 可能な限り`float`型を使用
2. **メモリコピーの最小化**: 参照渡しを積極的に使用
3. **行列ストレージ変換**: 必要な場合のみ変換
4. **動的メモリ割り当ての回避**: 固定サイズ配列を使用


