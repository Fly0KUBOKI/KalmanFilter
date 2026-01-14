# Unified Standalone API Migration

**更新日**: 2026年1月14日  
**対象**: C++ Kalman Filter ライブラリの完全な新構造化

---

## 変更内容

### 旧構造の廃止

**削除された API** （グローバル、後方互換性）:
```cpp
// ❌ 削除済み
uint8_t filter_init(void);
uint8_t filter_update(const SensorData& obs);
uint8_t filter_getState(State& out);
uint8_t filter_reset(void);
uint8_t filter_setType(FilterType t);

// ❌ グローバル変数削除
static FilterType g_type;
static FilterHandle g_handle;
static bool g_initialized;
```

### 新構造へ統一

**新しいハンドルベース API のみ** （複数インスタンス対応）:
```cpp
// ✅ 統一されたAPI
FilterHandle filter_create(FilterType type);
void filter_destroy(FilterHandle h);
uint8_t filter_init(FilterHandle h, const SensorData* init_data, uint32_t init_samples, float dt);
uint8_t filter_set_params(FilterHandle h, const Params& params);
uint8_t filter_set_gps_origin(FilterHandle h, double lat, double lon, double alt);
uint8_t filter_update(FilterHandle h, const SensorData& obs);
uint8_t filter_get_state(FilterHandle h, State& out);
uint8_t filter_reset(FilterHandle h);
uint8_t filter_is_initialized(FilterHandle h);
const char* filter_get_version(void);
```

### ファイル構成（新）
```
Lib/Common/
├── inc/standalone.hpp              ← ハンドルベースAPIのみ
└── src/standalone.cpp              ← FilterInstance実装のみ
                                      (グローバル変数なし)
```

---

## マイグレーション例

### Before（旧グローバルAPI）
```cpp
filter_setType(FILTER_ESKF);
if (filter_init() != 0) return 1;

filter_update(sensor_data);
filter_getState(state);
filter_reset();
```

### After（新ハンドルベースAPI）
```cpp
FilterHandle h = filter_create(FILTER_ESKF);
if (!h) return 1;

if (filter_init(h, nullptr, 0, 0.0f) != 0) {
    filter_destroy(h);
    return 1;
}

filter_update(h, sensor_data);
filter_get_state(h, state);
filter_reset(h);
filter_destroy(h);
```

### 複数インスタンスの例
```cpp
FilterHandle h1 = filter_create(FILTER_ESKF);
FilterHandle h2 = filter_create(FILTER_ESKF);

filter_init(h1, nullptr, 0, 0.0f);
filter_init(h2, nullptr, 0, 0.0f);

filter_update(h1, obs1);
filter_update(h2, obs2);

filter_get_state(h1, state1);
filter_get_state(h2, state2);

filter_destroy(h1);
filter_destroy(h2);
```

---

## 提供されるサンプル

| サンプル | API | 説明 |
|---------|-----|------|
| [main_simple.cpp](../examples/standalone/main_simple.cpp) | **新** | シンプルな単一フィルタの例 |
| [main_multi_instance.cpp](../examples/standalone/main_multi_instance.cpp) | **新** | マルチインスタンス＋ GPS 原点設定の例 |

---

## MATLAB への影響

✅ **MATLAB は影響なし**

- MATLAB は MEX（`mex_hybrid_filter`）経由でアクセス
- C++ スタンドアロン API の変更は直接影響しない
- MEX レイヤーは引き続き動作

---

## メリット

1. **シンプルな設計**
   - グローバル変数なし
   - 曖昧なオーバーロード解決なし

2. **マルチインスタンス対応**
   - 複数フィルタを並列実行可能
   - スレッド安全性の基盤

3. **柔軟な初期化**
   - センサーデータ、パラメータ、GPS原点を明示的に指定
   - 環境に合わせたカスタマイズが容易

4. **組み込み対応**
   - スタック割り当て可能（FilterInstance を static array で管理）
   - メモリ管理が明確

---

## ビルド＆テスト

### スタンドアロン実行ファイルビルド
```bash
cd kalman/cpp/examples/standalone

g++ -std=c++11 -I../../Lib \
    ../../Lib/Common/src/standalone.cpp \
    ../../Lib/ESKF/src/*.cpp \
    ../../Lib/MEUKF/src/*.cpp \
    ../../Lib/Matrix/src/*.cpp \
    main_simple.cpp -o kalman_simple

./kalman_simple
```

### マルチインスタンス例
```bash
g++ -std=c++11 -I../../Lib \
    ../../Lib/Common/src/standalone.cpp \
    ../../Lib/ESKF/src/*.cpp \
    ../../Lib/MEUKF/src/*.cpp \
    ../../Lib/Matrix/src/*.cpp \
    main_multi_instance.cpp -o kalman_multi

./kalman_multi
```

---

## 次のステップ

- [ ] CMakeLists.txt 作成（ビルド自動化）
- [ ] 組み込み環境向けサンプル（STM32）
- [ ] Python binding （ctypes で簡易 binding）
- [ ] 単体テストスイート
