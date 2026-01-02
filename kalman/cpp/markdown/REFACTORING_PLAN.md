# KalmanFilter C++ スタンドアロン化 & リファクタリング計画

**最終目標**: MATLABなしで C++ をコンパイル・実行できる統一 API を提供。複雑なヘッダー構造を整理し、メンテナンス性50%向上を目指す。

---

## 📋 全体構成

```
フェーズ1: Markdown ドキュメント整理（削減）
フェーズ2: ファイル構造の標準化（Lib/ 階層化）
フェーズ3: API 統一化（各フィルター共通インターフェース）
フェーズ4: インクルードパス統合（マスターヘッダー）
フェーズ5: スタンドアロン main() サンプル実装
フェーズ6: 型統一・最適化（uint8_t, float32 統一）
```

**進行期間**: 2-3 日想定（並列処理で短縮可能）

---

## 🔄 フェーズ 1: Markdown ドキュメント整理

### 目的
- 過時ドキュメント削除（不要な計画・分析ドキュメント）
- 必要なドキュメントを統合・簡潔化
- `markdown/` フォルダサイズ 50%削減

### 実施内容

#### 1.1 分類と削除リスト

**[過時] 削除対象（理由）**
- `LIBRARY_MIGRATION_PLAN.md` — 実施完了（2025-01-02）、参考不要
- `INCLUDE_PATH_FIX.md` — パス修正済み、現在のビルド問題なし
- `LIBRARY_MIGRATION_SUMMARY.md` — 報告完了、アーカイブ可
- `LIBRARY_MIGRATION_STATUS.md` — 進捗報告文書、完了後不要
- `LIBRARY_CLEANUP_PLAN.md` — 計画完了、`LIBRARY_CLEANUP_SUMMARY.md` に統合済み
- `LIBRARY_CLEANUP_COMPLETE.md` — 完了レポート、分析ドキュメントに統合可
- `MATRIX_LIBRARY_CLEANUP_COMPLETE.md` — 詳細完了レポート、不要
- `MEX_MIGRATION_COMPLETE.md` — 完了通知のみ、不要

**→ 計 8ファイル削除予定**

#### 1.2 統合対象

**[必要] 保持・統合**
- `CPP_INPUT_OUTPUT_SPEC.md` — MEX ↔ MATLAB 型仕様（メンテ用）
- `CPP_IMPLEMENTATION_OVERVIEW.md` — 現在の実装構造（リファクタリング時参考）
- `CPP_FILE_FUNCTION_INDEX.md` — ファイル一覧・関数索引（必須）
- `MEX_VS_PURE_CPP_ANALYSIS.md` — スタンドアロン化の方針記録
- `CPP_DEPENDENCIES.md` — 依存関係グラフ（参考）
- `STRUCTURE_AND_LIBRARIES.md` — ライブラリ構成（リファクタリング後更新）
- `CHOLESKY_DECOMPOSITION_INTEGRATION.md` — 行列計算の記録（参考）
- `Lib/README.md` — 現在のライブラリ説明（保持）
- `MEX/README.md` — MEX 実装原則（保持）

#### 1.3 新規作成

**`README_REFACTORING.md`** — 本計画の実行ガイド
- フェーズ別実施手順
- 進捗追跡スプレッドシート風
- リスク・対応策

### 完了基準

```bash
# Markdown ファイル数確認
ls kalman/cpp/markdown/*.md | wc -l
# 現在: ~15 → 目標: 8-9
```

---

## 🏗️ フェーズ 2: ファイル構造の標準化

### 目的
- 全フィルターを `Lib/*/inc/`, `Lib/*/src/` 階層に統一
- Matrix, Quaternion はヘッダーライブラリのまま（.hpp のみ）
- ファイル名を <15文字に簡潔化
- 各フォルダに依存関係を記載した `README.md` 配置

### 現状構造
```
Inc/
  Common/
  ESKF/       (9 .hpp files)
  MEUKF/      (4 .hpp files)
  EKF/        (2 .hpp files)
  UKF/        (3 .hpp files)
  KF/         (2 .hpp files)

Lib/
  Matrix/     (1 .hpp only)
  Quaternion/ (1 .hpp only)
  Common/     (1 .hpp)
  KalmanCore/ (未使用)

src/
  Common/
  ESKF/       (6 .cpp files)
  MEUKF/      (2 .cpp files)
  EKF/        (1 .cpp file)
  UKF/        (1 .cpp file)
```

### 目標構造
```
Lib/
  Matrix/
    fixed_matrix.hpp       ← ヘッダーのみ (現 Lib/Matrix/fixed_matrix.hpp)
  
  Quaternion/
    quat.hpp               ← ヘッダーのみ (現 Lib/Quaternion/quaternion_functions.hpp)
  
  Common/
    inc/
      types.hpp
      interface.hpp        ← Filter 基底クラス (新規)
      filter_mgmt.hpp      ← 発散検知・リセット
      math_utils.hpp       ← 数学ユーティリティ
      sensor_proc.hpp      ← センサー前処理・外れ値検知
    src/
      filter_mgmt.cpp
      sensor_proc.cpp
    README.md
  
  KF/
    inc/
      core.hpp             ← 現 Inc/KF/kf_core.hpp (リネーム)
    src/
      core.cpp             ← 現 src/KF/... (無い場合は作成)
    README.md
  
  EKF/
    inc/
      core.hpp             ← 現 Inc/EKF/ekf_core.hpp
      linear_update.hpp    ← 現 Inc/EKF/ekf_linear_update.hpp
    src/
      linear_update.cpp    ← 現 src/EKF/ekf_linear_update.cpp
    README.md
  
  UKF/
    inc/
      core.hpp             ← 現 Inc/UKF/ukf_core.hpp
      sigma_points.hpp     ← 現 Inc/UKF/ukf_sigma_points.hpp
      update.hpp           ← 現 Inc/UKF/ukf_update.hpp
    src/
      sigma_points.cpp     ← 現 src/UKF/ukf_sigma_points.cpp
    README.md
  
  ESKF/
    inc/
      core.hpp             ← 現 Inc/ESKF/eskf_core.hpp
      init.hpp             ← 現 Inc/ESKF/eskf_initializer.hpp
      runner.hpp           ← 現 Inc/ESKF/eskf_runner.hpp
      math.hpp             ← 現 Inc/ESKF/eskf_math.hpp
      sensor_update.hpp    ← 現 Inc/ESKF/eskf_sensor_updates.hpp
      postproc.hpp         ← 現 Inc/ESKF/eskf_postprocess.hpp
      helper.hpp           ← 現 Inc/ESKF/eskf_helper.hpp
      state.hpp            ← 現 Inc/ESKF/eskf_state.hpp
      filter.hpp           ← 現 Inc/ESKF/eskf_filter.hpp
    src/
      core.cpp
      init.cpp
      runner.cpp
      math.cpp
      sensor_update.cpp
      postproc.cpp
    README.md
  
  MEUKF/
    inc/
      core.hpp             ← 現 Inc/MEUKF/meukf_core.hpp
      types.hpp            ← 現 Inc/MEUKF/meukf_types.hpp (MEUKF特有型)
      unified.hpp          ← 現 Inc/MEUKF/unified_filter.hpp
    src/
      core.cpp
      unified.cpp
    README.md
  
  README.md                ← ライブラリ全体説明（更新）

Inc/
  kalman_all.hpp           ← マスターヘッダー (新規, フェーズ4)
  (将来的に削除、Lib/ に統合)

MEX/
  mex_run_eskf.cpp         ← MATLAB インターフェース（保持）
  Inc/
    mex_run_eskf_impl.hpp
    mex_type_conv.hpp      ← 型変換ユーティリティ
  README.md
```

### 実施手順

#### 2.1 フォルダ構造作成
```bash
cd kalman/cpp/Lib

# 各フィルターフォルダ作成
mkdir -p KF/{inc,src}
mkdir -p EKF/{inc,src}
mkdir -p UKF/{inc,src}
mkdir -p ESKF/{inc,src}
mkdir -p MEUKF/{inc,src}
mkdir -p Common/{inc,src}

# 既存 Common/inc 作成
mkdir -p Common/inc Common/src
```

#### 2.2 ファイル移動・リネーム
```bash
# KF
mv ../Inc/KF/kf_core.hpp Lib/KF/inc/core.hpp
mv ../Inc/KF/kalman_filter_core.hpp Lib/KF/inc/core_legacy.hpp  (削除検討)

# EKF
mv ../Inc/EKF/ekf_core.hpp Lib/EKF/inc/core.hpp
mv ../Inc/EKF/ekf_linear_update.hpp Lib/EKF/inc/linear_update.hpp
mv ../src/EKF/ekf_linear_update.cpp Lib/EKF/src/

# UKF
mv ../Inc/UKF/ukf_core.hpp Lib/UKF/inc/core.hpp
mv ../Inc/UKF/ukf_sigma_points.hpp Lib/UKF/inc/sigma_points.hpp
mv ../Inc/UKF/ukf_update.hpp Lib/UKF/inc/update.hpp
mv ../src/UKF/ukf_sigma_points.cpp Lib/UKF/src/

# ESKF
mv ../Inc/ESKF/eskf_core.hpp Lib/ESKF/inc/core.hpp
mv ../Inc/ESKF/eskf_initializer.hpp Lib/ESKF/inc/init.hpp
... (他 8ファイル同様)

# MEUKF
mv ../Inc/MEUKF/meukf_core.hpp Lib/MEUKF/inc/core.hpp
mv ../Inc/MEUKF/meukf_types.hpp Lib/MEUKF/inc/types.hpp
mv ../Inc/MEUKF/unified_filter.hpp Lib/MEUKF/inc/unified.hpp
mv ../src/MEUKF/meukf_core.cpp Lib/MEUKF/src/
mv ../src/MEUKF/unified_filter.cpp Lib/MEUKF/src/

# Common
mv ../Inc/Common/filter_interface.hpp Lib/Common/inc/interface.hpp
mv ../Inc/Common/filter_management.hpp Lib/Common/inc/filter_mgmt.hpp
mv ../src/Common/filter_management.cpp Lib/Common/src/
... (他も同様)
```

#### 2.3 フォルダ削除
```bash
rm -rf ../Inc/KF ../Inc/EKF ../Inc/UKF ../Inc/ESKF ../Inc/MEUKF
# ../Inc/Common/を ../Inc/kalman_filters.hpp のみに
```

#### 2.4 各フォルダに README.md 作成

**例: `Lib/ESKF/README.md`**
```markdown
# ESKF Library

## 概要
Error State Kalman Filter の実装。IMU + GPS + 磁気 の多センサー融合。

## ファイル構成
- `core.hpp/cpp` — predict, update の低レベル関数
- `init.hpp/cpp` — 初期化
- `runner.hpp/cpp` — ステップ実行エンジン
- `math.hpp/cpp` — 回転・クォータニオン計算
- `sensor_update.hpp/cpp` — センサー観測方程式
- `postproc.hpp/cpp` — オイラー角変換・後処理
- `state.hpp` — 状態構造体
- `filter.hpp` — フィルターラッパークラス

## 依存関係
- `Lib/Common/inc/interface.hpp` — フィルターベースクラス
- `Lib/Quaternion/quat.hpp` — クォータニオン演算
- `Lib/Matrix/fixed_matrix.hpp` — 行列演算

## 使用方法
```cpp
#include "Lib/ESKF/inc/filter.hpp"
kalman::ESKFFilter filter;
filter.init(...);
filter.update(sensor_data);
ESKFState state = filter.getState();
```
```

### 完了基準

```bash
# Inc/ フォルダが空（kalman_filters.hpp のみ）
find Inc -name "*.hpp" | wc -l  # 1以下

# Lib/ フォルダに全構造
find Lib -type f -name "*.hpp" -o -name "*.cpp" | wc -l  # ~60以上

# ファイル名確認（<15文字）
find Lib -name "*.hpp" -o -name "*.cpp" | while read f; do
  basename "$f" | awk 'length > 15 { print FILENAME ": " $0 }'
done
# 出力なし ← 成功
```

---

## 🎯 フェーズ 3: API 統一化

### 目的
- 各フィルター（KF, EKF, UKF, ESKF, MEUKF）に共通インターフェース定義
- スタンドアロン API 設計（main() で直接呼び出し可能）
- MATLAB MEX API と分離

### 3.1 共通フィルターベースクラス設計

**ファイル**: `Lib/Common/inc/interface.hpp`

```cpp
#pragma once

namespace kalman {

// =========== 共通型定義 ===========
struct SensorData {
  float accel[3];      // [m/s²]
  float gyro[3];       // [rad/s]
  float mag[3];        // [μT]
  float baro_alt;      // [m]
  double gps_lat;      // [deg]
  double gps_lon;      // [deg]
  double gps_alt;      // [m]
};

struct State {
  float p[3];          // 位置 [m]
  float v[3];          // 速度 [m/s]
  float q[4];          // クォータニオン [w,x,y,z]
  float euler[3];      // オイラー角 [roll, pitch, yaw] [rad]
  float ba[3];         // 加速度バイアス [m/s²]
  float bg[3];         // ジャイロバイアス [rad/s]
  float P[15*15];      // 共分散行列 [row-major]
};

struct Params {
  float g[3];          // 重力ベクトル [m/s²]
  float mag_ref[3];    // 基準磁気 [μT]
  float dt;            // サンプリング時間 [s]
  // ノイズ分散
  float noise_accel[3];
  float noise_gyro[3];
  float noise_ba[3];
  float noise_bg[3];
  float noise_mag[3];
  float noise_baro;
  double noise_gps[3];
};

// =========== 基底クラス ===========
class Filter {
public:
  virtual ~Filter() {}
  
  // 初期化（センサーデータから）
  virtual uint8_t init(const SensorData& obs, float static_time) = 0;
  
  // 1ステップ更新
  virtual uint8_t update(const SensorData& obs) = 0;
  
  // 現在状態取得
  virtual uint8_t getState(State& out) = 0;
  
  // パラメータ設定
  virtual uint8_t setParams(const Params& p) = 0;
  
  // リセット
  virtual uint8_t reset() = 0;
};

} // namespace kalman
```

### 3.2 各フィルターの実装テンプレート

**例: `Lib/ESKF/inc/filter.hpp`**

```cpp
#pragma once
#include "Lib/Common/inc/interface.hpp"

namespace kalman {

class ESKFFilter : public Filter {
private:
  ESKFState state_;  // 内部状態
  Params params_;    // パラメータ
  
public:
  ESKFFilter();
  ~ESKFFilter() override;
  
  uint8_t init(const SensorData& obs, float static_time) override;
  uint8_t update(const SensorData& obs) override;
  uint8_t getState(State& out) override;
  uint8_t setParams(const Params& p) override;
  uint8_t reset() override;
};

} // namespace kalman
```

**例: `Lib/KF/inc/filter.hpp`**（同構造）

```cpp
class KFFilter : public Filter {
  // 同じインターフェース
};
```

### 3.3 スタンドアロン API 設計

**ファイル**: `Lib/Common/inc/standalone.hpp`（新規）

```cpp
#pragma once
#include <cstdint>

// =========== グローバルフィルタンスタンス ===========
// MEX と異なり、グローバル状態管理（シングルトン）

namespace kalman {
  
// 初期化
uint8_t filter_init(void);

// 更新（単位: m/s², rad/s, μT）
uint8_t filter_update(float accel[3], float gyro[3], float mag[3]);

// 状態取得
uint8_t filter_getState(
  float angle[3],        // [rad] → [deg に変換]
  float rate[3],         // [rad/s]
  float pos[3],          // [m]
  float vel[3]           // [m/s]
);

// リセット
uint8_t filter_reset(void);

// オプション: フィルター選択
typedef enum {
  FILTER_KF,
  FILTER_EKF,
  FILTER_UKF,
  FILTER_ESKF,
  FILTER_MEUKF
} FilterType;

uint8_t filter_setType(FilterType t);

} // namespace kalman
```

**実装**: `Lib/Common/src/standalone.cpp`

```cpp
#include "Lib/Common/inc/standalone.hpp"
#include "Lib/ESKF/inc/filter.hpp"  // デフォルト

namespace kalman {

// グローバルインスタンス
static ESKFFilter g_filter;
static bool g_initialized = false;

uint8_t filter_init(void) {
  SensorData obs = {0};  // TODO: 初期観測データ取得
  static_time = 5.0f;    // 5秒静止初期化
  g_initialized = (g_filter.init(obs, static_time) == 0);
  return g_initialized ? 0 : 1;
}

uint8_t filter_update(float accel[3], float gyro[3], float mag[3]) {
  if (!g_initialized) return 1;
  
  SensorData obs = {0};
  memcpy(obs.accel, accel, sizeof(float)*3);
  memcpy(obs.gyro, gyro, sizeof(float)*3);
  memcpy(obs.mag, mag, sizeof(float)*3);
  
  return g_filter.update(obs);
}

uint8_t filter_getState(float angle[3], float rate[3], 
                         float pos[3], float vel[3]) {
  if (!g_initialized) return 1;
  
  State state;
  uint8_t ret = g_filter.getState(state);
  
  if (ret == 0) {
    // [rad] → [deg] 変換
    angle[0] = state.euler[0] * 180.0f / M_PI;
    angle[1] = state.euler[1] * 180.0f / M_PI;
    angle[2] = state.euler[2] * 180.0f / M_PI;
    
    memcpy(rate, state.bg, sizeof(float)*3);  // ジャイロバイアス？ 要確認
    memcpy(pos, state.p, sizeof(float)*3);
    memcpy(vel, state.v, sizeof(float)*3);
  }
  return ret;
}

uint8_t filter_reset(void) {
  g_initialized = false;
  return g_filter.reset();
}

} // namespace kalman
```

### 3.4 MATLAB MEX API との分離

**MEX API は現在のまま保持** (`mex_run_eskf()`)
- ハンドルベース（複数フィルターインスタンス管理）
- 動的型変換対応

**スタンドアロン API** （新規）
- グローバルシングルトン
- 固定型・固定メモリ確保
- main() から直接呼び出し可能

### 完了基準

```cpp
// 以下がコンパイル成功
#include "Lib/Common/inc/interface.hpp"
#include "Lib/ESKF/inc/filter.hpp"
#include "Lib/Common/inc/standalone.hpp"

kalman::Filter* f = new kalman::ESKFFilter();
kalman::filter_init();
kalman::filter_update({0,0,0}, {0,0,0}, {0,0,0});
```

---

## 🔗 フェーズ 4: インクルードパス統合

### 目的
- ヘッダー検索パスを 1 つのマスターヘッダーに集約
- ビルド時の `-I` フラグ数削減
- ユーザーは `#include "kalman/all.hpp"` のみ

### 4.1 マスターヘッダー作成

**ファイル**: `Inc/kalman_all.hpp`（新規）

```cpp
#pragma once

// ============================================
// KalmanFilter マスターヘッダー
// すべてのフィルター・ユーティリティを提供
// ============================================

// ===== 基盤ライブラリ =====
#include "Lib/Matrix/fixed_matrix.hpp"
#include "Lib/Quaternion/quat.hpp"

// ===== 共通型・インターフェース =====
#include "Lib/Common/inc/interface.hpp"
#include "Lib/Common/inc/filter_mgmt.hpp"
#include "Lib/Common/inc/math_utils.hpp"
#include "Lib/Common/inc/sensor_proc.hpp"

// ===== フィルター実装 =====
#include "Lib/KF/inc/filter.hpp"
#include "Lib/EKF/inc/filter.hpp"
#include "Lib/UKF/inc/filter.hpp"
#include "Lib/ESKF/inc/filter.hpp"
#include "Lib/MEUKF/inc/filter.hpp"

// ===== スタンドアロン API =====
#include "Lib/Common/inc/standalone.hpp"

// ===== バージョン情報 =====
#define KALMAN_VERSION "2.0.0"
#define KALMAN_BUILD_DATE __DATE__

namespace kalman {
  const char* version() { return KALMAN_VERSION; }
}
```

### 4.2 ビルド設定の統一

**CMakeLists.txt（新規）**

```cmake
cmake_minimum_required(VERSION 3.10)
project(KalmanFilter)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# インクルードパス一括指定
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/Inc
  ${CMAKE_CURRENT_SOURCE_DIR}/Lib
)

# ライブラリ（.cpp ファイル）を収集
file(GLOB_RECURSE SOURCES 
  "Lib/*/src/*.cpp"
  "Lib/Common/src/*.cpp"
)

# スタティックライブラリ化
add_library(kalman_static STATIC ${SOURCES})

# サンプル実行ファイル
add_executable(main_example examples/main_eskf.cpp)
target_link_libraries(main_example kalman_static)

# テスト（GTest）
enable_testing()
add_subdirectory(tests)
```

**build_mex.m（更新）**

```matlab
% MATLAB ビルドスクリプト
% CMake またはこのスクリプトで指定
include_dirs = {
  'Inc',
  'Lib'
};
source_files = {
  'Lib/*/src/*.cpp',
  'Lib/Common/src/*.cpp'
};

% MEX ビルド時に include_dirs を全て -I フラグで指定
```

### 完了基準

```bash
# マスターヘッダーのみインクルード可能
cat << 'EOF' > test.cpp
#include "Inc/kalman_all.hpp"
int main() {
  kalman::filter_init();
  return 0;
}
EOF

g++ -I Lib test.cpp Lib/*/src/*.cpp -o test
# コンパイル成功 ← 完了
```

---

## 💻 フェーズ 5: スタンドアロン main() サンプル実装

### 目的
- C++ スタンドアロンプログラムのサンプル提供
- CSV ファイル読み込み → フィルター実行 → 結果出力
- MATLAB `run_simulation.m` と等価の動作

### 5.1 ディレクトリ構造

```
examples/
  main_eskf.cpp          ← ESKF 単体実行
  main_meukf.cpp         ← MEUKF 単体実行
  data.csv               ← テストデータ（サンプル）
  CMakeLists.txt
  README.md
```

### 5.2 main_eskf.cpp（ESKF スタンドアロン）

```cpp
#include <cstdio>
#include <cstring>
#include "Inc/kalman_all.hpp"

// CSV 読み込み簡易実装
struct Obs {
  float accel[3], gyro[3], mag[3];
  float baro;
  double lat, lon, alt;
};

int readCSV(const char* filename, Obs* obs, int* count) {
  FILE* fp = fopen(filename, "r");
  if (!fp) return -1;
  
  *count = 0;
  while (fscanf(fp, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%lf,%lf,%lf",
                 &obs[*count].accel[0], &obs[*count].accel[1], 
                 &obs[*count].accel[2], &obs[*count].gyro[0],
                 &obs[*count].gyro[1], &obs[*count].gyro[2],
                 &obs[*count].mag[0], &obs[*count].mag[1],
                 &obs[*count].mag[2], &obs[*count].baro,
                 &obs[*count].lat, &obs[*count].lon, &obs[*count].alt) == 13) {
    (*count)++;
    if (*count >= 10000) break;  // 静的配列制限
  }
  fclose(fp);
  return 0;
}

int main(int argc, char* argv[]) {
  if (argc < 2) {
    fprintf(stderr, "Usage: %s <data.csv>\n", argv[0]);
    return 1;
  }
  
  // CSV 読み込み
  Obs obs_data[10000];
  int n_samples = 0;
  if (readCSV(argv[1], obs_data, &n_samples) != 0) {
    fprintf(stderr, "Failed to read CSV\n");
    return 1;
  }
  printf("Read %d samples\n", n_samples);
  
  // フィルター初期化
  kalman::ESKFFilter filter;
  kalman::SensorData init_obs = {0};
  memcpy(init_obs.accel, obs_data[0].accel, sizeof(float)*3);
  memcpy(init_obs.gyro, obs_data[0].gyro, sizeof(float)*3);
  memcpy(init_obs.mag, obs_data[0].mag, sizeof(float)*3);
  init_obs.baro_alt = obs_data[0].baro;
  init_obs.gps_lat = obs_data[0].lat;
  init_obs.gps_lon = obs_data[0].lon;
  init_obs.gps_alt = obs_data[0].alt;
  
  float static_time = 5.0f;
  if (filter.init(init_obs, static_time) != 0) {
    fprintf(stderr, "Filter init failed\n");
    return 1;
  }
  printf("Filter initialized\n");
  
  // メインループ
  FILE* out_fp = fopen("output.csv", "w");
  fprintf(out_fp, "time,px,py,pz,vx,vy,vz,roll,pitch,yaw\n");
  
  float time = 0.0f;
  for (int k = 1; k < n_samples; k++) {
    kalman::SensorData obs = {0};
    memcpy(obs.accel, obs_data[k].accel, sizeof(float)*3);
    memcpy(obs.gyro, obs_data[k].gyro, sizeof(float)*3);
    memcpy(obs.mag, obs_data[k].mag, sizeof(float)*3);
    obs.baro_alt = obs_data[k].baro;
    obs.gps_lat = obs_data[k].lat;
    obs.gps_lon = obs_data[k].lon;
    obs.gps_alt = obs_data[k].alt;
    
    if (filter.update(obs) != 0) {
      fprintf(stderr, "Update failed at k=%d\n", k);
      break;
    }
    
    kalman::State state;
    if (filter.getState(state) == 0) {
      fprintf(out_fp, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
              time, state.p[0], state.p[1], state.p[2],
              state.v[0], state.v[1], state.v[2],
              state.euler[0] * 180.0f / 3.14159f,
              state.euler[1] * 180.0f / 3.14159f,
              state.euler[2] * 180.0f / 3.14159f);
    }
    
    time += 0.01f;  // 10 Hz 仮定
    if (k % 1000 == 0) printf("Step %d / %d\n", k, n_samples);
  }
  fclose(out_fp);
  printf("Output written to output.csv\n");
  
  return 0;
}
```

### 5.3 ビルド＆実行

```bash
cd kalman/cpp/examples
cmake -DCMAKE_BUILD_TYPE=Release ..
make
./main_eskf ../GenerateData/sensor_data.csv
```

### 完了基準

```bash
# 実行結果が MATLAB run_simulation.m と ±0.1% 以内
cmp <(./main_eskf data.csv | cut -d, -f2) <(matlab_output.csv | cut -d, -f2)
# diff < 1e-3 ← 成功
```

---

## 🔤 フェーズ 6: 型統一・最適化

### 目的
- 全ファイルで float/double の混在を排除
- 200以下の整数は uint8_t/uint16_t に統一
- メモリ効率向上・バグ削減

### 6.1 型統一ガイドライン

**[允許型]**
```cpp
float         // 浮動小数点数（デフォルト）
double        // GPS座標など、精度が必須な場合のみ
uint8_t       // フラグ、カウンタ、ステータス（0-255）
uint16_t      // サンプル数、ステップ番号（0-65535）
int           // ループカウンタ（内部のみ）
```

**[禁止型]** (MEX 除く)
```cpp
std::vector<>   // 可変配列禁止 → static array or std::array
std::string     // 文字列禁止 → const char*
new/delete      // 動的メモリ禁止 → スタック・グローバル
int16_t, int32_t, long double, etc.  // 不要な型統一
```

### 6.2 修正対象ファイルリスト

各ファイルを grep で以下を検索・修正：

```bash
# double 使用箇所
grep -rn "double\s" Lib/ Inc/ | grep -v "gps_\|lat\|lon" | head -20

# std::vector 使用
grep -rn "std::vector" Lib/ Inc/ | head -10

# new/delete 使用
grep -rn "\bnew\b\|\bdelete\b" Lib/ Inc/ | head -10

# int/long の不適切使用
grep -rn "\bint\s\+" Lib/ Inc/ | grep -v "int k\|int main" | head -20
```

### 6.3 修正例

**Before:**
```cpp
struct FilterState {
  double p[3];            // ✗ double（不必要）
  double v[3];            // ✗ double
  std::vector<double> P;  // ✗ vector, double
  int iteration;          // ✗ int（過剰）
};
```

**After:**
```cpp
struct FilterState {
  float p[3];            // ✓ float
  float v[3];            // ✓ float
  float P[15*15];        // ✓ 静的配列
  uint8_t iteration;     // ✓ uint8_t（0-200程度）
};
```

### 6.4 MATLAB MEX との型変換

MEX インターフェース（`mex_type_conv.hpp`）で型変換を一元管理。
C++ 内部は float で統一。

```cpp
// MEX 受け取り（double）→ C++ (float) 変換
float* mxArrayToFloat(const mxArray* arr) {
  double* ptr = mxGetPr(arr);
  // → float に変換
}

// C++ (float) → MEX 返却（double） 変換
mxArray* floatToMxArray(const float* data, int m, int n) {
  mxArray* arr = mxCreateDoubleMatrix(m, n, mxREAL);
  // → double に変換して返却
}
```

### 完了基準

```bash
# double 使用箇所がなし（GPS座標除く）
grep -rn "double\s" Lib/ Inc/ | grep -v "gps_\|lat\|lon\|altitude" | wc -l
# 出力: 0 ← 成功

# std::vector/new/delete なし
grep -rn "std::vector\|new\b\|delete\b" Lib/ Inc/ | grep -v MEX | wc -l
# 出力: 0 ← 成功

# 全ファイル型統一確認
find Lib Inc -name "*.hpp" -o -name "*.cpp" | while read f; do
  echo "Checking $f"
  # 型チェック logic
done
```

---

## 📊 進捗追跡テーブル

| フェーズ | 内容 | 予定日 | 実績 | ステータス |
|--------|------|--------|------|----------|
| **0** | 計画策定 | 2026-01-02 | 2026-01-02 | ✅ 完了 |
| **1** | Markdown 整理 | 2026-01-02 | - | ⏳ 予定 |
| **2** | ファイル構造標準化 | 2026-01-03 | - | ⏳ 予定 |
| **3** | API 統一化 | 2026-01-04 | - | ⏳ 予定 |
| **4** | インクルード統合 | 2026-01-04 | - | ⏳ 予定 |
| **5** | main() サンプル | 2026-01-05 | - | ⏳ 予定 |
| **6** | 型統一・最適化 | 2026-01-05 | - | ⏳ 予定 |
| **検証** | 回帰テスト | 2026-01-06 | - | ⏳ 予定 |

---

## ⚠️ リスク・対応

| リスク | 発生確率 | 対応策 |
|--------|---------|--------|
| MEX ビルド失敗（include パス） | 中 | 各フェーズ後に `build_mex()` 実行・確認 |
| 既存コード互換性喪失 | 低 | ファイル移動時に Git 履歴保持（`git mv`） |
| 型統一による数値差異 | 低 | フェーズ6 後に `run_batch_10sets()` で回帰確認 |
| MATLAB 連携破断 | 中 | MEX API を現在のまま保持（互換性維持） |

---

## 🎯 最終成果物

### フェーズ完了後
```
Lib/
  ├─ Matrix/       (ヘッダーのみ)
  ├─ Quaternion/   (ヘッダーのみ)
  ├─ Common/{inc,src}
  ├─ KF/{inc,src}
  ├─ EKF/{inc,src}
  ├─ UKF/{inc,src}
  ├─ ESKF/{inc,src}
  ├─ MEUKF/{inc,src}
  └─ README.md     (更新)

Inc/
  └─ kalman_all.hpp  (マスターヘッダー)

examples/
  ├─ main_eskf.cpp
  ├─ main_meukf.cpp
  └─ CMakeLists.txt

markdown/
  ├─ README_REFACTORING.md   (本計画)
  ├─ CPP_INPUT_OUTPUT_SPEC.md
  ├─ CPP_FILE_FUNCTION_INDEX.md
  ├─ STRUCTURE_AND_LIBRARIES.md (更新)
  └─ (その他 5-6ファイル)

MEX/
  ├─ mex_run_eskf.cpp  (保持)
  └─ Inc/
     └─ mex_run_eskf_impl.hpp
```

### ユーザーAPI
```cpp
// MATLAB なし、C++ スタンドアロン実行
#include "Inc/kalman_all.hpp"

int main() {
  kalman::ESKFFilter filter;
  kalman::SensorData obs = {...};
  filter.init(obs, 5.0f);
  
  for (int k = 0; k < n_steps; k++) {
    filter.update(obs);
    kalman::State state;
    filter.getState(state);
  }
  return 0;
}
```

---

**最終版作成日**: 2026-01-02
**更新予定**: 各フェーズ完了時に進捗をこのドキュメントに追記
