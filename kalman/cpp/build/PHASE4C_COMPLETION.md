# Phase 4C 完了報告：発散チェックの統合

**完了日**: 2025-12-30  
**状態**: ✅ 完了

---

## 実施内容

### 変更ファイル

**`kalman/cpp/MEX/mex_eskf_do_update.cpp`**

1. **インクルードの追加**
   - `#include <vector>` を追加（`std::vector`使用のため）
   - `#include "../Inc/Common/Sensor/sensor_filter.hpp"` を追加
   - `using namespace common::sensor;` を追加

2. **グローバル変数の追加**
   - `static SensorFilterLib g_filter_lib;` を追加（発散チェック用）

3. **発散チェックの統合** (Line 258-288)
   - `mexCallMATLAB`で`mex_sensor_filter`の`divergence_check`を呼び出していた部分を削除
   - `g_filter_lib.divergence_guard.check_and_attenuate()`を直接呼び出すように変更

### 変更前（mexCallMATLAB経由）

```cpp
// Call divergence_guard.check_and_attenuate_update via mex_sensor_filter
mxArray* plhs_div[3];
mxArray* prhs_div[4];
prhs_div[0] = mxCreateString("divergence_check");
prhs_div[1] = mxCreateString(sensor_type);
prhs_div[2] = mxDuplicateArray(innov);
prhs_div[3] = vectorToMat(dx);

mexCallMATLAB(3, plhs_div, 4, prhs_div, "mex_sensor_filter");

// Get outputs: dx_out, should_skip, was_attenuated
Vector<15, float> dx_out;
if (!matToVector(plhs_div[0], dx_out)) {
    dx_out = dx;
}
should_skip = mxIsLogicalScalarTrue(plhs_div[1]);
bool was_attenuated = mxIsLogicalScalarTrue(plhs_div[2]);

// Cleanup divergence call
mxDestroyArray(plhs_div[0]);
mxDestroyArray(plhs_div[1]);
mxDestroyArray(plhs_div[2]);
mxDestroyArray(prhs_div[0]);
mxDestroyArray(prhs_div[1]);
mxDestroyArray(prhs_div[2]);
mxDestroyArray(prhs_div[3]);
```

### 変更後（C++直接実装）

```cpp
// Divergence check (C++ direct implementation)
// Get innov size
int innov_len = mxGetM(innov) * mxGetN(innov);
if (innov_len < 1) innov_len = 1;
if (innov_len > 3) innov_len = 3;

// Convert innov to FixedMatrix
cmath_fx::FixedMatrix innov_cm(innov_len, 1);
std::vector<float> innov_tmp(static_cast<size_t>(innov_len));
mex_conv::mxArrayToFloatArray(innov, innov_tmp.data(), static_cast<size_t>(innov_len));
for (int i = 0; i < innov_len; ++i) {
    innov_cm(i, 0) = innov_tmp[i];
}

// Convert dx to FixedMatrix
cmath_fx::FixedMatrix dx_cm(15, 1);
for (int i = 0; i < 15; ++i) {
    dx_cm(i, 0) = dx(i, 0);
}

// Call divergence check directly
bool was_attenuated = false;
bool should_skip_result = g_filter_lib.divergence_guard.check_and_attenuate(
    sensor_type, innov_cm, dx_cm, was_attenuated);

// Convert back to Vector
Vector<15, float> dx_out;
for (int i = 0; i < 15; ++i) {
    dx_out(i, 0) = dx_cm(i, 0);
}
should_skip = should_skip_result;
```

### 削除されたコード

- `mxArray`の作成・破棄処理（7個）
- `mexCallMATLAB`の呼び出し
- 約30行のコード削減

---

## 期待される効果

### `mexCallMATLAB`の呼び出し削減

- **統合前**: 1回（発散チェック）
- **統合後**: 0回（発散チェックはC++直接実装）
- **削減率**: 100%

### パフォーマンス向上

- `mexCallMATLAB`のオーバーヘッドが削減
- メモリ割り当て・解放の削減
- 型変換の最適化

---

## 注意事項

### グローバル変数の使用

- `g_filter_lib`は`static`変数として定義
- 各MEX呼び出し間で状態が保持される
- 発散チェックの履歴が維持される

### 型変換

- `innov`のサイズを動的に取得（1-3次元）
- `FixedMatrix`と`Vector`の適切な変換
- `double` ↔ `float`の変換

---

## Phase 4 統合の進捗

| フェーズ | 状態 | 完了日 |
|---------|------|--------|
| **Phase 4A: 前処理の統合** | ✅ 完了 | 2025-12-30 |
| **Phase 4B: 後処理の統合** | ✅ 完了（既に統合済み） | - |
| **Phase 4C: 発散チェックの統合** | ✅ 完了 | 2025-12-30 |

### `mexCallMATLAB`の呼び出し状況

- **統合前**: 5回
  - 前処理: 4回（accel, mag, baro, GPS）
  - 発散チェック: 1回
- **統合後**: 1回
  - 更新処理: 1回（`mex_eskf_do_update`内で`mex_meukf_step_v2`を呼び出し）

### 削減率

- **前処理**: 4回 → 0回（100%削減）
- **発散チェック**: 1回 → 0回（100%削減）
- **合計**: 5回 → 1回（80%削減）

---

## 次のステップ

### Phase 4D: 更新処理の統合（将来の検討事項）

- `mex_meukf_step_v2`を直接C++実装に置き換える
- MEUKFアルゴリズムを維持する必要がある
- リスクが高いため、現時点では実施しない

---

## テスト

### ビルドテスト

```matlab
cd kalman/cpp/build
build_mex({'mex_eskf_do_update', 'mex_run_eskf'})
```

### 精度テスト

```matlab
clear mex
run_simulation(42, true)
```

### バッチテスト

```matlab
run_batch_10sets()
```

---

## 注意事項

- 発散チェックの動作が正しいことを確認
- 精度が維持されることを確認（Position RMSE < 1.0m）
- グローバル変数の状態管理が適切であることを確認

---

**最終更新**: 2025-12-30


