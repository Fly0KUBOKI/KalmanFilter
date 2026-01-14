# Standalone API リファクタリング計画

**作成日**: 2026年1月14日  
**目的**: C++ Libを独立したライブラリとして、STM32などの組み込み環境で使用可能にする

---

## 現状分析

### ✅ 良い点（既存の設計）

1. **API定義は存在する**
   - [Lib/Common/inc/standalone.hpp](../kalman/cpp/Lib/Common/inc/standalone.hpp) にシンプルなAPI定義あり
   - [Lib/Common/src/standalone.cpp](../kalman/cpp/Lib/Common/src/standalone.cpp) に実装あり

2. **MEX層との分離は達成されている**
   - MEXフォルダには型変換とmexFunctionのみ
   - アルゴリズムはすべてLib/配下に実装

3. **モジュール化されたライブラリ構造**
   - Matrix、Quaternion、Sensor などが独立したヘッダーファイル
   - 各レイヤーが明確に分離（7層設計）

### ❌ 問題点（改善が必要）

#### 1. **API設計が不完全**
```cpp
// 現在のAPI（standalone.hpp）
uint8_t filter_init(void);                      // ❌ センサーデータを引数で渡せない
uint8_t filter_update(const SensorData& obs);   // ✅ OK
uint8_t filter_getState(State& out);            // ✅ OK
uint8_t filter_reset(void);                     // ✅ OK
uint8_t filter_setType(FilterType t);           // ✅ OK
```

**問題**:
- `filter_init()` がセンサーデータを受け取れない（内部でゼロ初期化）
- パラメータ設定APIがない（ノイズ共分散、重力ベクトルなど）
- GPSオリジン設定APIがない
- dtやstatic_timeをハードコード（5.0秒固定）

#### 2. **インターフェース構造体の二重定義**
```cpp
// interface.hpp に2つの異なるセンサーデータ構造体が存在
struct SensorData {         // Version 1（standalone用、単純）
  float accel[3];
  float gyro[3];
  // ...
};

struct SensorInput {        // Version 2（内部用、高機能）
  Vector3 accel;
  bool accel_updated;
  // ...
};
```

**問題**:
- どちらを使うべきか不明確
- 変換コストが発生する

#### 3. **ドキュメント不足**
- standalone APIの使用例がない
- 組み込み環境向けのサンプルコードがない
- READMEにAPIの記載がない

#### 4. **ビルド方法が不明確**
```bash
# これが可能か？現在は不明
g++ -I Lib/ Lib/**/*.cpp main.cpp -o standalone_app
```

**問題**:
- MEXなしでのビルド方法が文書化されていない
- CMakeLists.txt や Makefileがない
- 依存関係が不明

#### 5. **メモリ管理がグローバル変数依存**
```cpp
// standalone.cpp
static Filter* g_filter = nullptr;  // ❌ グローバル変数
```

**問題**:
- マルチインスタンス不可（複数フィルタを並列実行できない）
- スレッドセーフでない
- 組み込み環境での動的メモリ割り当て（new/delete）

---

## 改善提案

### Phase 1: API設計の改善 ⚡ 優先度: 高

#### 1.1 完全なAPI定義

```cpp
// 新しいAPI設計案（standalone_v2.hpp）
namespace kalman {

// ハンドル型（不透明ポインタ）
typedef void* FilterHandle;

// 初期化API（パラメータを明示的に渡す）
FilterHandle filter_create(FilterType type);
uint8_t filter_init(FilterHandle h, const SensorData* init_data, uint32_t init_samples, float dt);
uint8_t filter_set_params(FilterHandle h, const Params& params);
uint8_t filter_set_gps_origin(FilterHandle h, double lat, double lon, double alt);

// 更新API（既存）
uint8_t filter_update(FilterHandle h, const SensorData& obs);
uint8_t filter_get_state(FilterHandle h, State& out);

// リセット・解放API
uint8_t filter_reset(FilterHandle h);
void filter_destroy(FilterHandle h);

// ユーティリティAPI
uint8_t filter_is_initialized(FilterHandle h);
const char* filter_get_version(void);

} // namespace kalman
```

**改善点**:
- ハンドルベースで複数インスタンス対応
- 初期化時にセンサーデータとパラメータを渡せる
- GPSオリジン設定を明示的に

#### 1.2 インターフェース構造体の統一

```cpp
// SensorData を SensorInput に統一
// または SensorData を完全版に拡張
struct SensorData {
    // IMU
    float accel[3];
    float gyro[3];
    uint8_t accel_valid;
    uint8_t gyro_valid;
    
    // Mag
    float mag[3];
    uint8_t mag_valid;
    
    // GPS
    double gps_lat;
    double gps_lon;
    double gps_alt;
    uint8_t gps_valid;
    
    // Baro
    float pressure;
    uint8_t baro_valid;
    
    // Timestamp
    float dt;
};
```

### Phase 2: ドキュメント整備 📖 優先度: 高

#### 2.1 Lib/README.md の作成

独立したライブラリとしてのREADMEを作成:
- API一覧と説明
- 使用例（MATLAB MEX、スタンドアロン、組み込み）
- ビルド方法
- 依存関係

#### 2.2 サンプルコード作成

```
kalman/cpp/examples/
├── standalone/
│   ├── main_simple.cpp          # 最小限の使用例
│   ├── main_from_csv.cpp        # CSVからセンサーデータ読込
│   └── Makefile                 # スタンドアロンビルド
├── embedded/
│   ├── stm32_example.c          # STM32向けサンプル
│   └── README.md                # 組み込み環境向けガイド
└── README.md
```

### Phase 3: ビルドシステム整備 🔧 優先度: 中

#### 3.1 CMakeLists.txt作成

```cmake
# kalman/cpp/CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project(KalmanFilter)

# ライブラリターゲット
add_library(kalman_filter STATIC
    Lib/Common/src/*.cpp
    Lib/ESKF/src/*.cpp
    Lib/MEUKF/src/*.cpp
    # ...
)

target_include_directories(kalman_filter PUBLIC Lib/)

# サンプル実行ファイル
add_executable(example_standalone examples/standalone/main_simple.cpp)
target_link_libraries(example_standalone kalman_filter)
```

#### 3.2 Makefileの追加

組み込み環境向けのシンプルなMakefile:
```makefile
# examples/standalone/Makefile
CXX = g++
CXXFLAGS = -std=c++11 -I../../Lib
SOURCES = main_simple.cpp ../../Lib/**/*.cpp
TARGET = kalman_standalone

$(TARGET): $(SOURCES)
	$(CXX) $(CXXFLAGS) -o $@ $^
```

### Phase 4: 組み込み最適化 🚀 優先度: 低

#### 4.1 動的メモリ削除

```cpp
// 現在（動的メモリ使用）
Filter* g_filter = new ESKFFilter();

// 改善案（静的メモリ）
struct FilterInstance {
    ESKFFilter filter;
    bool initialized;
};

static FilterInstance g_instances[MAX_FILTERS];

FilterHandle filter_create(FilterType type) {
    // g_instances から未使用のものを返す
}
```

#### 4.2 浮動小数点最適化

- float/doubleの使い分けを明確化
- GPS座標変換後は float で統一
- ARM Cortex-M4F の FPU 活用

---

## 実装優先順位

### 🔴 Phase 1: 即座に実施（1-2日）
1. ✅ 現状分析（完了）
2. ⏳ standalone_v2.hpp の設計と実装
3. ⏳ main_simple.cpp サンプル作成
4. ⏳ Lib/README.md 作成

### 🟡 Phase 2: 短期（1週間）
5. ⏳ examples/ フォルダの充実
6. ⏳ CMakeLists.txt 作成
7. ⏳ docs/README.md 更新（APIセクション追加）

### 🟢 Phase 3: 中長期（2週間以降）
8. ⏳ 組み込み環境サンプル（STM32）
9. ⏳ 動的メモリ削除リファクタリング
10. ⏳ 単体テストスイート作成

---

## 期待される成果

### 開発者体験の向上
- ✅ 5分でAPIを理解できるドキュメント
- ✅ 10行でフィルタを実行できるサンプルコード
- ✅ 3コマンドでビルドできる環境

### 適用範囲の拡大
- ✅ MATLAB以外での使用（Python、C++アプリなど）
- ✅ 組み込み環境（STM32、ESP32など）
- ✅ リアルタイムOS（FreeRTOS、Zephyrなど）

### コード品質の向上
- ✅ インターフェース定義の明確化
- ✅ 依存関係の文書化
- ✅ テストカバレッジの向上

---

## 次のステップ

1. **即座に**: standalone_v2.hpp の設計レビュー
2. **今週中**: main_simple.cpp の実装とテスト
3. **来週**: ドキュメント整備とサンプル追加

このリファクタリングにより、KalmanFilterプロジェクトは**真の汎用ライブラリ**に進化します。
