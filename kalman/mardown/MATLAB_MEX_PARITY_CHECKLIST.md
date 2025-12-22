# MATLAB/MEX パリティチェックリスト

**作成日**: 2025年12月22日  
**目的**: MATLABとMEX（C++）実装間の不一致を防止するためのチェックリスト

---

## 🔴 重要なルール

### 1. センサーフィルタのパリティ

| チェック項目 | MATLAB実装 | C++/MEX実装 | 確認状況 |
|------------|-----------|-------------|----------|
| **重力ノルム検証** | `[8.5, 10.5]` 範囲チェック | `gravity_range_min_`, `gravity_range_max_` | ✅ 修正済 (2025/12/22) |
| 外れ値検出 (3σ) | `SensorAccelFilter.m` L89 | `sensor_filter.hpp` L760 | ✅ |
| EMAフィルタ係数 | `ema_alpha = 0.3` | `accel_filter.set_alpha(0.3f)` | ✅ |
| ノイズ履歴サイズ | `history_size = 20` | `MAX_HISTORY = 20` | ✅ |

### 2. 数値精度

| 項目 | MATLAB | C++ | 注意事項 |
|------|--------|-----|---------|
| 浮動小数点型 | `double` (64bit) | 多くが `float` (32bit) | **PHASE 5 で修正予定** |
| クォータニオン正規化 | 自動 | 明示的に実施 | C++側で実施、MATLAB側で重複禁止 |
| 共分散対称化 | `P = (P + P')/2` | `regularize_covariance()` | 両方で実施 |

---

## 🟡 過去に発生した問題と対策

### Issue #1: 重力ノルム検証の欠落 (2025/12/22)

**症状**:
- MEXモード: Roll/Pitch RMSE ≈ 1.5-1.7 deg
- MATLABモード: Roll/Pitch RMSE ≈ 0.27-0.29 deg
- 約5-6倍の精度劣化

**根本原因**:
1. **重力ノルム検証のコメントアウト**: C++ `sensor_filter.hpp` の `filter_accel()` で重力ノルム検証がコメントアウトされていた
2. **外れ値検出の初期化ロジック不一致**: `OutlierDetector.detect()` で履歴が空の場合の処理がMATLAB側と異なっていた

**修正内容**:

1. **重力ノルム検証の有効化** ([sensor_filter.hpp#L740-L752](../cpp/include/Common/Sensor/sensor_filter.hpp#L740)):
```cpp
// 加速度ノルムが [gravity_range_min_, gravity_range_max_] 範囲外なら外れ値として棄却
if (a_norm < gravity_range_min_ || a_norm > gravity_range_max_) {
    is_outlier = true;
    return accel_filter.get_value();
}
```

2. **外れ値検出の初期化ロジック修正** ([sensor_filter.hpp#L248-L270](../cpp/include/Common/Sensor/sensor_filter.hpp#L248)):
```cpp
// 修正前: 履歴が5個未満の場合、noise_std = min_std = 0.1 で固定
//        → 最初のデータが外れ値と誤判定される

// 修正後: 履歴が空の場合、noise_std = residual_norm とする
//        → MATLAB側と同一の動作（最初のデータは外れ値と判定されない）
if (count_ == 0) {
    noise_std = fmaxf(residual_norm, min_std);
}
```

**教訓**:
1. テスト用のコメントアウトは必ず元に戻す
2. 初期化直後の動作もMATLAB側と一致させる必要がある
3. 境界条件（履歴が空、履歴が少ない）のテストを必ず実施

---

### Issue #2: OutlierDetector の noise_std 計算と履歴更新の不一致 (2025/12/22)

**症状**:
- 最初の5サンプルは正常に処理される
- 6サンプル目以降、99.5%が外れ値として誤検出
- Roll/Pitch RMSE ≈ 1.5-1.7 deg (MATLABモードでは ≈ 0.27 deg)

**根本原因**:
1. **noise_std の下限計算不一致**: 履歴が5件以上ある場合、C++では `noise_std = max(noise_std, min_std)` だったが、MATLABでは `noise_std = max(noise_std, residual_norm / 3.0)` を使用
2. **外れ値の履歴追加**: C++では外れ値も履歴に追加していたが、MATLABでは外れ値は履歴に追加しない

**修正内容**:

1. **noise_std の下限を修正** ([sensor_filter.hpp#L279](../cpp/include/Common/Sensor/sensor_filter.hpp#L279)):
```cpp
// 修正前
noise_std = fmaxf(noise_std, min_std);

// 修正後: MATLAB parity - residual_norm / 3.0 を下限に含める
noise_std = fmaxf(noise_std, fmaxf(residual_norm / 3.0f, min_std));
```

2. **外れ値の履歴追加を禁止** ([sensor_filter.hpp#L285-L293](../cpp/include/Common/Sensor/sensor_filter.hpp#L285)):
```cpp
// 修正前: 無条件で履歴に追加
if (count_ < MAX_HISTORY) {
    history_[count_++] = residual_norm;
}

// 修正後: MATLAB parity - 外れ値でない場合のみ履歴に追加
if (!is_outlier) {
    if (count_ < MAX_HISTORY) {
        history_[count_++] = residual_norm;
    }
}
```

**検証方法**:
```matlab
% 外れ値検出率を確認
addpath('cpp/bin'); clear mex;
mex_sensor_filter('reset');  
sensor = readtable('GenerateData/sensor_data.csv');
n_outlier = 0;
for i=1:1000
    a_meas = [sensor.accel_x(i); sensor.accel_y(i); sensor.accel_z(i)];
    [~, is_outlier] = mex_sensor_filter('accel', a_meas, [0;0;0]);
    if is_outlier; n_outlier = n_outlier + 1; end
end
fprintf('Outlier rate: %.1f%% (expect ~0.3%%)\n', n_outlier/10);
```

**教訓**:
1. 外れ値検出ロジックは履歴管理も含めて完全に一致させる必要がある
2. 「通常ケース」と「境界ケース」の両方でMATLABとの動作一致を確認する
3. 外れ値検出率が大きく異なる場合は直接テストして比較する

---

## 🟢 検証手順

### 新しい機能追加時

1. **MATLAB側の実装を確認**
   ```matlab
   % 関連するMATLABファイルを特定
   grep -r "function_name" kalman/KF/Utils/
   ```

2. **C++側の対応する実装を確認**
   ```bash
   grep -r "function_name" kalman/cpp/include/
   ```

3. **パラメータ値の一致を確認**
   - 閾値、係数、範囲などの数値定数
   - デフォルト値

4. **バッチテストで検証**
   ```matlab
   % MATLABモード
   setenv('FORCE_MATLAB_FILTERS', '1');
   run_batch_10sets();
   
   % MEXモード
   setenv('FORCE_MATLAB_FILTERS', '0');
   run_batch_10sets();
   
   % 結果比較
   % Roll/Pitch RMSEが両モードで同等 (< 0.05 deg差) であることを確認
   ```

### コードレビューチェックリスト

- [ ] C++側のすべての検証ロジックがMATLAB側と一致している
- [ ] `NOTE:`, `TODO:`, `FIXME:`, `一時的`, `コメントアウト` がないか確認
- [ ] テスト用に無効化したコードが元に戻されている
- [ ] 数値定数がMATLAB側と一致している
- [ ] 浮動小数点の精度が適切（double推奨）

---

## 📁 関連ファイル

### MATLAB側
- [SensorAccelFilter.m](../KF/Utils/SensorAccelFilter.m) - 加速度フィルタ（重力検証あり）
- [AccelFilter.m](../KF/Utils/AccelFilter.m) - 加速度フィルタ（MEX呼び出しのみ）
- [SensorFilters.m](../KF/Utils/SensorFilters.m) - MEXラッパークラス
- [SensorFilter.m](../KF/Utils/SensorFilter.m) - ファクトリークラス

### C++/MEX側
- [sensor_filter.hpp](../cpp/include/Common/Sensor/sensor_filter.hpp) - センサーフィルタ実装
- [mex_sensor_filter.cpp](../cpp/mex/mex_sensor_filter.cpp) - MEXエントリポイント

### ビルド
- [build_mex.m](../cpp/build/build_mex.m) - MEXビルドスクリプト

---

## 🔧 ビルド/テスト手順

```matlab
% 1. MEXビルド
cd kalman/cpp/build
build_mex('mex_sensor_filter')  % または build_mex() で全体ビルド

% 2. キャッシュクリア（必須）
clear mex

% 3. テスト実行
cd ../..
setenv('FORCE_MATLAB_FILTERS', '0');
run_batch_10sets();

% 4. 結果確認
% Results/log/batch_10sets_log_mex_*.txt を確認
% Roll/Pitch RMSE が 0.27-0.30 deg 程度であれば正常
```

---

## 📝 更新履歴

| 日付 | 内容 |
|------|------|
| 2025/12/22 | 初版作成、重力ノルム検証問題の記録 |
| 2025/12/22 | **Issue #2追加**: OutlierDetector の `noise_std` 計算と履歴更新ロジックの修正 |
