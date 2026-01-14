# Kalman Filter C++ Library

**バージョン**: 1.0.0  
**更新日**: 2026年1月14日  
**ライセンス**: MIT（予定）

高精度慣性航法システム（INS）のためのC++ライブラリ。MATLAB MEX、スタンドアロンアプリケーション、組み込みシステム（STM32など）で使用可能。

---

## 🚀 クイックスタート

### スタンドアロンアプリケーション（最小限の例）

```cpp
#include "Common/inc/standalone.hpp"
#include <iostream>

int main() {
    using namespace kalman;
    
    // 1. フィルタタイプを設定
    filter_setType(FILTER_ESKF);
    
    // 2. 初期化
    if (filter_init() != 0) {
        std::cerr << "Filter initialization failed\n";
        return 1;
    }
    
    // 3. センサーデータを準備
    SensorData obs;
    obs.accel[0] = 0.0f; obs.accel[1] = 0.0f; obs.accel[2] = -9.81f;
    obs.gyro[0] = 0.0f; obs.gyro[1] = 0.0f; obs.gyro[2] = 0.0f;
    obs.mag[0] = 30.0f; obs.mag[1] = 0.0f; obs.mag[2] = -50.0f;
    obs.baro_alt = 100.0f;
    obs.gps_lat = 35.6812; obs.gps_lon = 139.7671; obs.gps_alt = 40.0;
    
    // 4. フィルタ更新
    if (filter_update(obs) != 0) {
        std::cerr << "Filter update failed\n";
        return 1;
    }
    
    // 5. 状態を取得
    State state;
    if (filter_getState(state) != 0) {
        std::cerr << "Get state failed\n";
        return 1;
    }
    
    // 6. 結果を表示
    std::cout << "Position: [" << state.p[0] << ", " 
              << state.p[1] << ", " << state.p[2] << "]\n";
    std::cout << "Euler: [" << state.euler[0] << ", " 
              << state.euler[1] << ", " << state.euler[2] << "] deg\n";
    
    // 7. クリーンアップ
    filter_reset();
    
    return 0;
}
```

### ビルド方法

```bash
# スタンドアロン実行ファイル（Linux/Mac）
g++ -std=c++11 -I. \
    Common/src/*.cpp ESKF/src/*.cpp MEUKF/src/*.cpp \
    Matrix/src/*.cpp \
    main.cpp -o kalman_app

# 実行
./kalman_app
```

---

## 📚 ライブラリ構造

### 7層アーキテクチャ

```
Lib/
├── LAYER 1: ユーティリティ（独立、依存なし）
│   ├── Matrix/              固定サイズ行列テンプレート
│   ├── Quaternion/          四元数演算（[w,x,y,z]）
│   └── Common/inc/Math/     数学関数、統計、Mahalanobis距離
│
├── LAYER 2: センサー処理
│   └── Common/inc/Sensor/   外れ値検出、フィルタリング、前処理
│
├── LAYER 3: フィルタ管理
│   └── Common/inc/          共分散管理、ZUPT、発散検出
│
└── LAYER 4: フィルタ実装（ホットパス）
    ├── ESKF/                Error-State Kalman Filter
    └── MEUKF/               Multiplicative Extended UKF
```

### 主要モジュール

| モジュール | ファイル | 説明 |
|-----------|---------|------|
| **API** | `Common/inc/standalone.hpp` | 公開API定義 |
| **インターフェース** | `Common/inc/interface.hpp` | SensorData, State, Params構造体 |
| **行列演算** | `Matrix/fixed_matrix.hpp` | 行列、ベクトル、Cholesky分解 |
| **四元数** | `Quaternion/quaternion_functions.hpp` | 正規化、乗算、Euler変換 |
| **ESKFコア** | `ESKF/inc/eskf_core.hpp` | 予測・更新・状態積分 |
| **センサー** | `Common/inc/Sensor/sensor_filter.hpp` | 外れ値検出、ロバスト推定 |

---

## 🔌 API リファレンス

### フィルタ初期化・設定

#### `filter_setType(FilterType t)`
フィルタタイプを設定。

**パラメータ**:
- `t`: `FILTER_ESKF`, `FILTER_MEUKF`, `FILTER_UKF`, `FILTER_EKF`, `FILTER_KF`

**戻り値**: `0` 成功

#### `filter_init(void)`
フィルタを初期化（現在の実装では内部でゼロセンサーデータで初期化）。

**戻り値**: `0` 成功、`1` 失敗

**⚠️ 注意**: 将来のバージョンではセンサーデータとパラメータを引数で渡せるよう改善予定（[STANDALONE_API_REFACTORING_PLAN.md](../../../docs/STANDALONE_API_REFACTORING_PLAN.md)参照）。

### フィルタ更新・状態取得

#### `filter_update(const SensorData& obs)`
センサーデータでフィルタを更新。

**パラメータ**:
- `obs`: センサー観測値（加速度、ジャイロ、磁気、GPS、気圧）

**戻り値**: `0` 成功、`1` 失敗

#### `filter_getState(State& out)`
現在の推定状態を取得。

**パラメータ**:
- `out`: 出力先（位置、速度、四元数、バイアスなど）

**戻り値**: `0` 成功、`1` 失敗

### クリーンアップ

#### `filter_reset(void)`
フィルタをリセット（メモリ解放）。

**戻り値**: `0` 成功

---

## 📦 データ構造

### SensorData（入力）

```cpp
struct SensorData {
    float accel[3];      // 加速度 [m/s²] (body frame)
    float gyro[3];       // 角速度 [rad/s] (body frame)
    float mag[3];        // 磁気 [µT] (body frame)
    float baro_alt;      // 気圧高度 [m]
    double gps_lat;      // GPS緯度 [deg]
    double gps_lon;      // GPS経度 [deg]
    double gps_alt;      // GPS高度 [m]
};
```

### State（出力）

```cpp
struct State {
    float p[3];          // 位置 [m] (ENU frame)
    float v[3];          // 速度 [m/s] (ENU frame)
    float q[4];          // 四元数 [w, x, y, z] (body→ENU)
    float euler[3];      // オイラー角 [deg] (roll, pitch, yaw)
    float ba[3];         // 加速度バイアス [m/s²]
    float bg[3];         // ジャイロバイアス [rad/s]
    float P[15*15];      // 共分散行列（column-major）
};
```

### Params（設定）

```cpp
struct Params {
    float g[3];              // 重力ベクトル [m/s²] (通常 [0, 0, -9.81])
    float mag_ref[3];        // 地磁気参照 [µT]
    float dt;                // サンプリング周期 [s]
    float noise_accel[3];    // 加速度ノイズ [m/s²]
    float noise_gyro[3];     // ジャイロノイズ [rad/s]
    float noise_ba[3];       // 加速度バイアスノイズ
    float noise_bg[3];       // ジャイロバイアスノイズ
    float noise_mag[3];      // 磁気ノイズ [µT]
    float noise_baro;        // 気圧ノイズ [m]
    double noise_gps[3];     // GPSノイズ [m] (ENU)
};
```

---

## 🛠️ 使用例

### 1. MATLAB MEX経由（現在の主要用途）

```matlab
% MATLABからMEX経由で使用
handle = mex_run_eskf('init', obs, static_time, dt);
mex_run_eskf('step', handle, obs, k);
state = mex_run_eskf('get_state', handle);
mex_run_eskf('free', handle);
```

詳細は [kalman/run_simulation.m](../../run_simulation.m) を参照。

### 2. スタンドアロンC++アプリケーション

**CSVファイルからセンサーデータを読み込み**:
```cpp
// 将来実装予定: examples/standalone/main_from_csv.cpp
// CSV読込 → フィルタ更新 → 結果をCSVに保存
```

### 3. 組み込みシステム（STM32）

**FreeRTOS上でのリアルタイム実行**:
```c
// 将来実装予定: examples/embedded/stm32_example.c
// IMUからセンサーデータ取得 → フィルタ更新 → UART出力
```

---

## 🔍 依存関係

### 必須
- **C++11以上**（`std::memset`, テンプレート、`nullptr`など）
- **標準ライブラリ**: `<cmath>`, `<cstring>`, `<cstdint>`, `<iostream>`（サンプルのみ）

### オプション
- **MATLAB R2018b以上**（MEX使用時）
- **ARM CMSIS-DSP**（組み込み最適化時）

### 外部ライブラリ不要
- ✅ Eigen不使用（独自の`fixed_matrix.hpp`）
- ✅ Boost不使用
- ✅ OpenCV不使用

---

## ⚙️ ビルドシステム

### 現在対応
- ✅ MATLAB MEX（`build_mex.m`）
- ✅ 手動g++コンパイル

### 今後対応予定
- ⏳ CMake（クロスプラットフォーム）
- ⏳ Makefile（組み込み環境）
- ⏳ ARM GCC（STM32）

詳細は [STANDALONE_API_REFACTORING_PLAN.md](../../../docs/STANDALONE_API_REFACTORING_PLAN.md) を参照。

---

## 📊 パフォーマンス

### MATLAB MEX実行時（2026-01-11測定）
- **位置RMSE**: 0.32m（10セット平均）
- **姿勢RMSE**: 0.31°/0.31°/0.79°（Roll/Pitch/Yaw）
- **実行時間**: < 25秒/10,000ステップ
- **バイナリサイズ**: 339 KB（MinGW64）

### メモリ使用量（概算）
- **フィルタ状態**: ~5 KB（ESKFState + 共分散）
- **スタック**: ~10 KB（予測・更新関数）
- **動的メモリ**: ~1 KB（Filter*インスタンス）

---

## 🚧 制限事項と既知の問題

### 現在のAPI設計の制限
1. **グローバル変数依存**: 複数フィルタインスタンス不可
2. **初期化パラメータ**: `filter_init()`が引数を受け取らない
3. **GPSオリジン**: 内部で自動設定（手動設定不可）

→ **Phase 1で改善予定**（[リファクタリング計画](../../../docs/STANDALONE_API_REFACTORING_PLAN.md)）

### 座標系
- **入力**: Body frame（IMU、磁気）、ECEF（GPS）
- **出力**: ENU frame（East-North-Up、GPS原点基準）
- **四元数**: Body → ENU 回転を表現

### 浮動小数点
- **内部計算**: `float`（32bit）
- **GPS座標**: `double`（64bit）→ 変換後は`float`

---

## 📖 関連ドキュメント

| ドキュメント | 説明 |
|-------------|------|
| [docs/README.md](../../../docs/README.md) | プロジェクト全体の概要 |
| [docs/CPP_ARCHITECTURE.md](../../../docs/CPP_ARCHITECTURE.md) | C++アーキテクチャ詳細 |
| [docs/LIB_STRUCTURE.md](../../../docs/LIB_STRUCTURE.md) | Lib層の構造と全関数リスト |
| [docs/STANDALONE_API_REFACTORING_PLAN.md](../../../docs/STANDALONE_API_REFACTORING_PLAN.md) | API改善計画 |
| [docs/CODING_STANDARDS.md](../../../docs/CODING_STANDARDS.md) | コーディング規約 |
| [MEX/README.md](../../MEX/README.md) | MEX層の設計原則 |

---

## 🤝 貢献

バグ報告、機能要求、ドキュメント改善は歓迎です。

### 開発ガイドライン
1. **コーディング規約**: [CODING_STANDARDS.md](../../../docs/CODING_STANDARDS.md)
2. **型使用規則**: GPS以外は`float`、状態ベクトルは`[p,v,q,ba,bg]`順
3. **四元数**: 必ず`[w,x,y,z]`、正規化は`cquat::normalize_quat<T>()`

---

## 📜 ライセンス

MIT License（予定）

---

**更新履歴**:
- 2026-01-14: 初版作成、API制限事項の明記
- 今後: API v2実装後に更新予定
