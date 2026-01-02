# C++関数リファレンス

## MEX関数

### mex_run_eskf

**ファイル**: `MEX/mex_run_eskf.cpp`

**エントリーポイント**: `mexFunction`

**コマンド**:
- `init`: ESKF初期化
- `step`: ESKFステップ実行
- `get_state`: 状態取得
- `free`: メモリ解放
- `meukf_step`: MEUKFステップ実行
- `sensor_filter_reset_zero`: センサーフィルターリセット（ゼロ）
- `sensor_filter_reset`: センサーフィルターリセット
- `sensor_filter_update`: センサーフィルター更新

#### init
```cpp
uint64_t do_init(const mxArray* obs, double static_time, double dt)
```
- **入力**: 
  - `obs`: MATLAB構造体（センサーデータ）
  - `static_time`: 静止時間 [s]
  - `dt`: サンプリング時間 [s]
- **出力**: ハンドル（uint64_t）

#### step
```cpp
void do_step(ESKFState* s, const mxArray* obs, int k)
```
- **入力**:
  - `s`: ESKF状態
  - `obs`: MATLAB構造体（センサーデータ）
  - `k`: ステップ番号（1-based）
- **出力**: なし（状態を更新）

#### get_state
```cpp
mxArray* do_get_state(ESKFState* s)
```
- **入力**: `s`: ESKF状態
- **出力**: MATLAB構造体（p, v, q, euler, ba, bg, P）すべてfloat型

#### meukf_step
```cpp
void do_meukf_step(const mxArray* m_prev_state, const mxArray* m_sensor, 
                   const mxArray* m_params, mxArray*& out_new_state, 
                   mxArray*& out_dbg_out, mxArray*& out_dbg_output)
```
- **入力**:
  - `m_prev_state`: 前回状態（MATLAB構造体）
  - `m_sensor`: センサーデータ（MATLAB構造体）
  - `m_params`: パラメータ（MATLAB構造体）
- **出力**:
  - `out_new_state`: 更新後状態（MATLAB構造体）
  - `out_dbg_out`: デバッグ情報（MATLAB構造体）
  - `out_dbg_output`: 詳細デバッグ情報（MATLAB構造体）

## ESKF関数

### ESKFCore

**ファイル**: `src/ESKF/eskf_core.cpp`

#### integrate_nominal
```cpp
void integrate_nominal(Vector3& p, Vector3& v, Vector4& q, Vector3& ba, Vector3& bg,
                       const Vector3& a_meas, const Vector3& w_meas,
                       Scalar dt, const Vector3& g,
                       const Vector3& gyro_noise_threshold,
                       const Vector3& accel_noise_threshold)
```
- **入力/出力**: 状態変数（参照渡し）
- **入力**: 測定値、時間、重力、閾値
- **処理**: RK2積分でノミナル状態を更新

#### predict_covariance
```cpp
void predict_covariance(const Matrix15x15& P, const Vector4& q, 
                       const Vector3& a_meas, const Vector3& ba,
                       const Vector3& w_meas, const Vector3& bg, 
                       const Matrix15x15& Q, Scalar dt,
                       Matrix15x15& P_new)
```
- **入力**: 共分散、状態、測定値、プロセスノイズ、時間
- **出力**: 予測共分散
- **処理**: `P_new = F * P * F' + Q`

#### update_accel
```cpp
void update_accel(Vector4& q, const Vector3& a_meas, Scalar scale_factor = 1.0)
```
- **入力/出力**: クォータニオン（参照渡し）
- **入力**: 加速度測定値、スケールファクター
- **処理**: 加速度からRoll/Pitchを更新

#### update_mag
```cpp
void update_mag(Vector4& q, Matrix15x15& P,
                const Vector3& m_meas, const Vector3& m_world,
                const Matrix3x3& R_mag,
                cmath_fx::Matrix<15, 3, Scalar>& K_out, Vector15& dx_out)
```
- **入力/出力**: クォータニオン、共分散（参照渡し）
- **入力**: 磁気測定値、ワールド磁場、観測ノイズ
- **出力**: カルマンゲイン、誤差状態
- **処理**: 磁気計更新（ESKF）

#### update_gps
```cpp
void update_gps(Vector3& p, Vector3& v, Matrix15x15& P,
                const Vector3& gps_pos, const Vector3& gps_origin,
                const Matrix3x3& R_gps,
                cmath_fx::Matrix<15, 3, Scalar>& K_out, Vector15& dx_out)
```
- **入力/出力**: 位置、速度、共分散（参照渡し）
- **入力**: GPS位置、原点、観測ノイズ
- **出力**: カルマンゲイン、誤差状態
- **処理**: GPS更新（線形観測）

#### update_baro
```cpp
void update_baro(Vector3& p, Matrix15x15& P, Scalar altitude,
                 const Vector3& gps_origin, Scalar R_baro,
                 cmath_fx::Matrix<15, 1, Scalar>& K_out, Vector15& dx_out)
```
- **入力/出力**: 位置、共分散（参照渡し）
- **入力**: 気圧高度、原点、観測ノイズ
- **出力**: カルマンゲイン、誤差状態
- **処理**: 気圧計更新（スカラー観測）

#### update_zupt
```cpp
void update_zupt(const Vector3& v_in, const Matrix15x15& P_in,
                 Vector3& v_out, Matrix15x15& P_out)
```
- **入力**: 速度、共分散
- **出力**: 更新後速度、共分散
- **処理**: Zero Velocity Update（速度=0を観測）

#### compute_adaptive_Q
```cpp
void compute_adaptive_Q(const Matrix15x15& Q_nominal,
                        const Vector3& a_meas, const Vector3& w_meas,
                        Matrix15x15& Q_adapted)
```
- **入力**: ノミナルプロセスノイズ、測定値
- **出力**: 適応プロセスノイズ
- **処理**: 動的ノイズ調整

### ESKFRunner

**ファイル**: `src/ESKF/eskf_runner.cpp`

#### predict
```cpp
void predict(ESKFState* s, const double* a_meas, const double* w_meas)
```
- **入力/出力**: ESKF状態（参照渡し）
- **入力**: 加速度、角速度測定値（double型）
- **処理**: 予測ステップ（ノミナル状態積分、共分散予測、後処理）

### ESKFInitializer

**ファイル**: `src/ESKF/eskf_initializer.cpp`

#### initialize_eskf_from_matlab
```cpp
ESKFState* initialize_eskf_from_matlab(const mxArray* obs, double static_time, double dt)
```
- **入力**: MATLAB構造体、静止時間、サンプリング時間
- **出力**: 初期化されたESKF状態
- **処理**: 静止データから初期状態を推定

### ESKFSensorUpdates

**ファイル**: `src/ESKF/eskf_sensor_updates.cpp`

#### update_accel_sensor
```cpp
SensorUpdateResult update_accel_sensor(ESKFState* s,
                                        const Vector<3, float>& a_meas,
                                        SensorFilterLib& filter_lib)
```
- **入力**: ESKF状態、加速度測定値、フィルターライブラリ
- **出力**: 更新結果
- **処理**: 加速度センサー更新（前処理、更新、後処理）

#### update_mag_sensor
```cpp
SensorUpdateResult update_mag_sensor(ESKFState* s,
                                      const Vector<3, float>& m_meas,
                                      SensorFilterLib& filter_lib)
```
- **入力**: ESKF状態、磁気測定値、フィルターライブラリ
- **出力**: 更新結果
- **処理**: 磁気センサー更新

#### update_baro_sensor
```cpp
SensorUpdateResult update_baro_sensor(ESKFState* s, double pressure,
                                       SensorFilterLib& filter_lib)
```
- **入力**: ESKF状態、気圧、フィルターライブラリ
- **出力**: 更新結果
- **処理**: 気圧センサー更新

#### update_gps_sensor
```cpp
SensorUpdateResult update_gps_sensor(ESKFState* s, double lat, double lon, double alt,
                                      SensorFilterLib& filter_lib)
```
- **入力**: ESKF状態、GPS座標、フィルターライブラリ
- **出力**: 更新結果
- **処理**: GPSセンサー更新

## MEUKF関数

### MEUKFCore

**ファイル**: `src/MEUKF/meukf_core.cpp`

#### step
```cpp
void step(const MEUKFInput& input, MEUKFOutput& output)
```
- **入力**: MEUKF入力（前回状態、センサーデータ、パラメータ）
- **出力**: MEUKF出力（更新後状態、デバッグ情報）
- **処理**: MEUKFメインループ（予測→更新）

#### predict
```cpp
void predict(State& state, const SensorData& sensor, const Params& params)
```
- **入力/出力**: 状態（参照渡し）
- **入力**: センサーデータ、パラメータ
- **処理**: 状態予測（クォータニオン積分、位置・速度積分、共分散予測）

#### update_accel_meukf
```cpp
void update_accel_meukf(State& state, const Vector3& a_meas, 
                        const Params& params, MEUKFOutput& output)
```
- **入力/出力**: 状態（参照渡し）
- **入力**: 加速度測定値、パラメータ
- **出力**: デバッグ情報
- **処理**: UKFによる加速度更新（2D観測、Roll/Pitchのみ）

#### update_mag_meukf
```cpp
void update_mag_meukf(State& state, const Vector3& m_meas,
                      const Params& params, MEUKFOutput& output)
```
- **入力/出力**: 状態（参照渡し）
- **入力**: 磁気測定値、パラメータ
- **出力**: デバッグ情報
- **処理**: UKFによる磁気更新（3D観測）

#### update_gps
```cpp
void update_gps(State& state, const Vector3& gps_meas,
                const Params& params, MEUKFOutput& output)
```
- **入力/出力**: 状態（参照渡し）
- **入力**: GPS測定値、パラメータ
- **出力**: デバッグ情報
- **処理**: 線形カルマンフィルターによるGPS更新

#### update_baro
```cpp
void update_baro(State& state, float alt_baro,
                 const Params& params, MEUKFOutput& output)
```
- **入力/出力**: 状態（参照渡し）
- **入力**: 気圧高度、パラメータ
- **出力**: デバッグ情報
- **処理**: 線形カルマンフィルターによる気圧更新

#### update_zupt
```cpp
void update_zupt(State& state, const Params& params, MEUKFOutput& output)
```
- **入力/出力**: 状態（参照渡し）
- **入力**: パラメータ
- **出力**: デバッグ情報
- **処理**: ZUPT更新（速度=0を観測）

## 共通関数

### FilterManagement

**ファイル**: `src/Common/filter_management.cpp`

#### check_divergence
```cpp
bool check_divergence(const cmath_fx::Matrix<15, 15, float>& P)
```
- **入力**: 共分散行列
- **出力**: 発散フラグ
- **処理**: NaN/Inf、大きな分散をチェック

#### normalize_covariance
```cpp
void normalize_covariance(cmath_fx::Matrix<15, 15, float>& P)
```
- **入力/出力**: 共分散行列（参照渡し）
- **処理**: 最大分散制限を適用

#### check_state_divergence
```cpp
bool check_state_divergence(const cmath_fx::Vector<3, float>& p,
                            const cmath_fx::Vector<3, float>& v,
                            const cmath_fx::Vector<4, float>& q,
                            const cmath_fx::Vector<3, float>& ba,
                            const cmath_fx::Vector<3, float>& bg,
                            const cmath_fx::Matrix<15, 15, float>& P)
```
- **入力**: 状態変数、共分散
- **出力**: 発散フラグ
- **処理**: 状態発散チェック

### SensorPreprocessor

**ファイル**: `src/Common/Sensor/sensor_preprocessor.cpp`

#### preprocess_accel
```cpp
PreprocessResult preprocess_accel(const cmath_fx::Vector<3, float>& a_meas,
                                   const cmath_fx::Vector<3, float>& prev_a,
                                   double buffer_tolerance)
```
- **入力**: 加速度測定値、前回値、許容誤差
- **出力**: 前処理結果（変更検知、外れ値検知）
- **処理**: 変更検知、外れ値チェック

#### preprocess_mag
```cpp
PreprocessResult preprocess_mag(const cmath_fx::Vector<3, float>& m_meas,
                                 const cmath_fx::Vector<3, float>& prev_m,
                                 double buffer_tolerance)
```
- **入力**: 磁気測定値、前回値、許容誤差
- **出力**: 前処理結果
- **処理**: 変更検知

#### preprocess_baro
```cpp
double preprocess_baro(double pressure)
```
- **入力**: 気圧 [Pa]
- **出力**: 高度 [m]
- **処理**: 標準大気モデルで高度変換

#### preprocess_gps
```cpp
PreprocessResult preprocess_gps(double lat, double lon, double alt,
                                 const cmath_fx::Vector<3, float>& origin,
                                 double buffer_tolerance)
```
- **入力**: GPS座標、原点、許容誤差
- **出力**: 前処理結果（メートル単位の位置）
- **処理**: GPS座標→メートル変換、変更検知

## 型変換関数

### mex_conv

**ファイル**: `MEX/mex_type_conv.hpp`, `Inc/MEX/mex_type_conversion.hpp`

#### mxArrayToFloatArray
```cpp
void mxArrayToFloatArray(const mxArray* arr, float* out, std::size_t n)
```
- **入力**: MATLAB配列、出力バッファ、サイズ
- **出力**: float配列
- **制約**: single型のみ受け付け（GPS以外）

#### mxGetScalarAsFloat
```cpp
float mxGetScalarAsFloat(const mxArray* a)
```
- **入力**: MATLABスカラー
- **出力**: float値
- **制約**: single型のみ受け付け（GPS以外）

