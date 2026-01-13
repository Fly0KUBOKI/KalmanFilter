# KalmanFilter C++ コーディング規約

**作成日**: 2026-01-11  
**目的**: 型の不整合によるコンパイラ依存問題の再発防止

---

## 🎯 基本方針

**このプロジェクトは、MSVC と MinGW の両方で完全に同一の数値結果を保証する。**

そのために、以下の規約を**厳守**する：

1. **型の一貫性**: GPS関連のみ `double`、その他は全て `float`
2. **不要な型変換の禁止**: `float → double → float` の往復変換を行わない
3. **明示的な型変換**: 必要な場合のみ `static_cast<T>()` を使用

---

## 📏 型の使用規則

### Rule 1: 浮動小数点型の使い分け

#### `double` を使用する場合（GPS関連のみ）

```cpp
// ✅ GOOD: GPS座標は double
double gps_lat, gps_lon, gps_alt;      // 度・メートル
double gps_origin[3];                   // GPS原点 [lat, lon, alt]

// GPS → ENU 変換も double で計算
double x_m = (gps_lon - origin_lon) / (9.0e-6 / cos_lat);
double y_m = (gps_lat - origin_lat) / 9.0e-6;
double z_m = gps_alt - origin_alt;
```

**理由**:
- GPS座標は地球規模の値（-180°～180°、-90°～90°）
- メートル変換時に桁落ちを防ぐため `double` が必要
- GPSセンサーの仕様が `double` （IEEE 754 64bit）

#### `float` を使用する場合（その他全て）

```cpp
// ✅ GOOD: 状態ベクトルは float
float p[3], v[3], q[4], ba[3], bg[3];

// センサー測定値も float
float accel[3], gyro[3], mag[3], pressure;

// 共分散行列も float
float P[15*15], Q_nominal[15*15];

// スカラー値も float
float dt, sigma_accel, sigma_gyro;
```

**理由**:
- 相対座標（ENU）は数十メートル程度なので `float` で十分
- 四元数は正規化されるため、精度は問題なし
- メモリ使用量を半減（15x15行列で900 bytes vs 1800 bytes）

---

### Rule 2: 型変換の禁止パターン

#### ❌ BAD: 不要な往復変換

```cpp
// ❌ BAD: float → double → float の往復変換
float x = 1.0f;
double temp = static_cast<double>(x);  // 精度は向上しない
float y = static_cast<float>(temp);    // 2回目の丸め誤差が発生

// ❌ BAD: 構造体への代入で型が異なる
Vector<4, float> quat_float;
normalize_quat(quat_float);
q[0] = static_cast<double>(quat_float(0,0));  // ← float q[4] なのに double 経由
```

**問題**:
- MinGW と MSVC で最適化の挙動が異なる
- `-ffloat-store` により、MinGW は中間結果をメモリに書き戻す
- → 丸め誤差が2回発生し、10^-9 レベルの差が生じる

#### ✅ GOOD: 型を統一する

```cpp
// ✅ GOOD: float のまま処理
float x = 1.0f;
float y = x;  // 型変換なし

// ✅ GOOD: 構造体のメンバー型と一致
Vector<4, float> quat_float;
normalize_quat(quat_float);
q[0] = static_cast<float>(quat_float(0,0));  // float → float (一貫性)
```

---

### Rule 3: GPS → ENU 変換時の型変換

#### ✅ GOOD: GPS (double) → ENU (float) の変換

```cpp
// GPS原点（double）
double gps_origin_lat = 35.681236;  // 東京
double gps_origin_lon = 139.767125;
double gps_origin_alt = 0.0;

// GPS測定値（double）
double gps_lat = 35.682;
double gps_lon = 139.768;
double gps_alt = 50.0;

// ENU変換（double で計算）
double cos_lat = std::cos(gps_origin_lat * DEG2RAD);
double x_m = (gps_lon - gps_origin_lon) / (9.0e-6 / cos_lat);
double y_m = (gps_lat - gps_origin_lat) / 9.0e-6;
double z_m = gps_alt - gps_origin_alt;

// 最後に float に変換（1回だけ）
float p_enu[3];
p_enu[0] = static_cast<float>(x_m);  // ← これは許容（GPS→ENU変換）
p_enu[1] = static_cast<float>(y_m);
p_enu[2] = static_cast<float>(z_m);
```

**ポイント**:
- GPS座標系では `double` で計算（精度保証）
- ENU座標系に変換後、`float` に1回だけ変換
- **往復変換は行わない**

---

## 🚫 禁止事項リスト

### 1. 不要な `static_cast` の連鎖

```cpp
// ❌ BAD
float x = static_cast<float>(static_cast<double>(1.0f));

// ❌ BAD
double y = static_cast<double>(static_cast<float>(2.0));

// ✅ GOOD
float x = 1.0f;
double y = 2.0;
```

---

### 2. 中間変数の型が最終型と異なる

```cpp
// ❌ BAD
float result;
double temp = compute_something();  // double で計算
result = static_cast<float>(temp);  // float に変換

// ✅ GOOD
float result = compute_something_float();  // 最初から float
```

---

### 3. リテラルの型が曖昧

```cpp
// ❌ BAD
float x = 1.0;   // double リテラル → float に暗黙変換
float y = 3.14;  // double リテラル

// ✅ GOOD
float x = 1.0f;  // float リテラル
float y = 3.14f;
```

---

## ✅ 推奨パターン

### Pattern 1: 初期化時の型統一

```cpp
// ❌ BAD
double accel_mean_x_d, accel_mean_y_d, accel_mean_z_d;
compute_mean_3d(..., &accel_mean_x_d, &accel_mean_y_d, &accel_mean_z_d);
float accel_mean_x = static_cast<float>(accel_mean_x_d);  // 型変換

// ✅ GOOD
float accel_mean_x, accel_mean_y, accel_mean_z;
compute_mean_3d_float(..., &accel_mean_x, &accel_mean_y, &accel_mean_z);  // 最初から float
```

---

### Pattern 2: ベクトル・行列の型テンプレート

```cpp
// ✅ GOOD: テンプレート引数で型を明示
Vector<3, float> p_float;   // float 版
Matrix<15, 15, float> P;    // float 版

// GPS関連のみ double
Vector<3, double> gps_lla;  // [lat, lon, alt]
```

---

### Pattern 3: 関数引数の型を明確に

```cpp
// ❌ BAD: 型が曖昧
void update_state(double* p, double* v);  // 内部で float に変換？

// ✅ GOOD: 型を明確に
void update_state_float(float* p, float* v);
void update_gps_double(double lat, double lon, double alt);
```

---

## 🔍 レビューチェックリスト

### コードレビュー時に確認すべき項目

#### [ ] 型の一貫性
- GPS関連以外で `double` を使っていないか？
- `float → double → float` の往復変換がないか？
- リテラルに `f` サフィックスが付いているか？

#### [ ] 不要な型変換
- `static_cast<float>(static_cast<double>(x))` のような連鎖がないか？
- 構造体メンバーと異なる型で計算していないか？

#### [ ] テンプレート引数
- `Vector<N, T>`, `Matrix<N, M, T>` の `T` が適切か？
- GPS関連のみ `double`、その他は `float` か？

#### [ ] コンパイラ警告
- `-Wconversion` 警告が出ていないか？
- `/W4` で警告が出ていないか？

---

## 🧪 テスト要件

### 型の一貫性テスト

```cpp
// test_type_consistency.cpp
#include <cassert>
#include <type_traits>

void test_eskf_state_types() {
    ESKFState s;
    
    // 状態ベクトルは float
    static_assert(std::is_same<decltype(s.p[0]), float>::value, "p must be float");
    static_assert(std::is_same<decltype(s.v[0]), float>::value, "v must be float");
    static_assert(std::is_same<decltype(s.q[0]), float>::value, "q must be float");
    static_assert(std::is_same<decltype(s.ba[0]), float>::value, "ba must be float");
    static_assert(std::is_same<decltype(s.bg[0]), float>::value, "bg must be float");
    
    // GPS origin のみ double
    static_assert(std::is_same<decltype(s.gps_origin[0]), double>::value, "gps_origin must be double");
    
    // 共分散行列は float
    static_assert(std::is_same<decltype(s.P[0]), float>::value, "P must be float");
    static_assert(std::is_same<decltype(s.Q_nominal[0]), float>::value, "Q_nominal must be float");
}
```

---

### コンパイラ一貫性テスト

**MATLAB スクリプト**: `test_compiler_consistency.m`

```matlab
function test_compiler_consistency()
    fprintf('=== Compiler Consistency Test ===\n\n');
    
    % MSVCでビルド & テスト
    fprintf('Building with MSVC...\n');
    select_mex_compiler('msvc');
    build_mex();
    clear mex;
    
    fprintf('Running batch test with MSVC...\n');
    run_batch_10sets();
    msvc_results = readtable('Results/batch_10sets_summary.csv');
    
    % MinGWでビルド & テスト
    fprintf('\nBuilding with MinGW...\n');
    select_mex_compiler('mingw');
    build_mex();
    clear mex;
    
    fprintf('Running batch test with MinGW...\n');
    run_batch_10sets();
    mingw_results = readtable('Results/batch_10sets_summary.csv');
    
    % 差分チェック
    fprintf('\n=== Results Comparison ===\n');
    diff_pos = max(abs(msvc_results.PosX_RMSE_m - mingw_results.PosX_RMSE_m));
    diff_att = max(abs(msvc_results.Roll_RMSE_deg - mingw_results.Roll_RMSE_deg));
    
    fprintf('Position RMSE diff: %.6f m (threshold: 0.01 m)\n', diff_pos);
    fprintf('Attitude RMSE diff: %.6f deg (threshold: 0.01 deg)\n', diff_att);
    
    % 許容差分: 1cm 位置、0.01 deg 姿勢
    if diff_pos < 0.01 && diff_att < 0.01
        fprintf('\n✅ PASS: Compiler consistency check passed\n');
    else
        error('❌ FAIL: Compiler consistency check failed');
    end
end
```

---

## 📚 参考資料

### 浮動小数点演算の基礎

- **IEEE 754**: 浮動小数点数の標準仕様
- **float (32bit)**: 仮数部 23bit、指数部 8bit、符号部 1bit
- **double (64bit)**: 仮数部 52bit、指数部 11bit、符号部 1bit

### 精度の比較

| 型 | 仮数部精度 | 有効桁数 | 範囲 |
|------|----------|---------|------|
| `float` | 23bit | 約7桁 | ±3.4e38 |
| `double` | 52bit | 約15桁 | ±1.7e308 |

### なぜ GPS は double が必要か？

```
緯度・経度の桁数例:
  東京タワー: 35.658581, 139.745433
  
float の精度: 約7桁
  → 35.65858 (小数点以下5桁) → 誤差 ±1m
  
double の精度: 約15桁
  → 35.658581234567 (小数点以下12桁) → 誤差 ±0.01mm
```

---

## 🏁 まとめ

### 必ず守るべき3つのルール

1. **GPS関連のみ `double`、その他は全て `float`**
2. **不要な型変換（特に往復変換）を行わない**
3. **構造体メンバーと同じ型で計算する**

### コードレビューの重点項目

1. `static_cast<float>(static_cast<double>(...))` を見つけたら即座に修正
2. GPS→ENU変換以外で `double` を使っていたら警告
3. リテラルに `f` サフィックスがない場合は警告

### テストの実施

- 新機能追加時: `test_compiler_consistency()` を実行
- コミット前: `run_batch_10sets()` で回帰テスト
- 型変更時: `static_assert` でコンパイル時チェック

---

**作成者**: GitHub Copilot  
**最終更新**: 2026-01-11  
**関連ドキュメント**: [COMPILER_ISSUE_ROOT_CAUSE_FINAL.md](COMPILER_ISSUE_ROOT_CAUSE_FINAL.md)
