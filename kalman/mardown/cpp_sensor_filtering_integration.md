# C++センサーフィルタリング統合完了報告

## 作業概要

### 問題
- KF/Utils/ (16ファイル) と Common/Math/QuaternionLib.m を誤って削除
- C++への移行が完了していないのに削除してしまった
- 結果: NaN発散が500回以上発生 (Step 15000-20000)

### 解決策
1. **削除したファイルをgitで復元** ✅
   - `git checkout HEAD -- Common/Math/QuaternionLib.m`
   - `git checkout HEAD -- KF/Utils/`
   - 全17ファイル復元完了

2. **C++側にセンサーフィルタリング統合** ✅
   - 場所: `cpp/MEUKF/meukf_core.cpp`
   - 既存実装を活用: `cpp/include/Common/Sensor/sensor_filter.hpp`

## 実装詳細

### 変更ファイル
1. **cpp/MEUKF/meukf_core.cpp**
   - グローバル`SensorFilterLib`インスタンス追加
   - `MEUKFCore::step()`関数にセンサーフィルタリング統合
   - 実装内容:
     * EMAフィルタ (加速度、磁気計、気圧)
     * Biquad ローパスフィルタ (ジャイロ)
     * Alpha-Betaフィルタ (GPS)
     * 外れ値検出 (加速度、磁気計)
     * 共分散正則化 (DivergenceGuard)

### コード構造

```cpp
// cpp/MEUKF/meukf_core.cpp

#include "../Common/Sensor/sensor_filter.hpp"

namespace meukf {

// グローバルフィルタインスタンス (呼び出し間で永続化)
static common::sensor::SensorFilterLib g_sensor_filters;

void MEUKFCore::step(const MEUKFInput& input, MEUKFOutput& output) {
    // ========== センサーフィルタリング ==========
    SensorData filtered_sensor = input.sensor;
    
    // 1. 加速度フィルタリング (EMA + 外れ値検出)
    if (input.sensor.update_accel) {
        // ... EMAフィルタ適用
    }
    
    // 2. ジャイロフィルタリング (Biquad ローパス)
    if (input.sensor.update_gyro) {
        // ... Biquadフィルタ適用
    }
    
    // 3. 磁気計フィルタリング (EMA + 外れ値検出)
    if (input.sensor.update_mag) {
        // ... EMAフィルタ適用
    }
    
    // 4. GPS フィルタリング (Alpha-Beta)
    if (input.sensor.update_gps) {
        // ... Alpha-Betaフィルタ適用
    }
    
    // 5. 気圧フィルタリング (EMA)
    if (input.sensor.update_baro) {
        // ... EMAフィルタ適用
    }
    
    // ========== フィルタ実行 (フィルタリング済みデータ使用) ==========
    predict(output.new_state, filtered_sensor, input.params);
    update_accel_meukf(...);
    update_mag_meukf(...);
    update_gps(...);
    update_baro(...);
    update_zupt(...);
    
    // ========== 共分散正則化 ==========
    g_sensor_filters.divergence_guard.regularize_covariance(P_mat);
}
}
```

### センサーフィルタリング詳細

| センサー | フィルタ種類 | パラメータ | 外れ値検出 |
|---|---|---|---|
| **加速度** | EMA | α=0.3 | ✅ 有効 |
| **ジャイロ** | Biquad Lowpass | fc=50Hz | ❌ なし |
| **磁気計** | EMA | α=0.2 | ✅ 有効 |
| **GPS** | Alpha-Beta | α=0.5, β=0.1 | ❌ なし |
| **気圧** | EMA | α=0.4 | ❌ なし |

### DivergenceGuard機能
- 共分散行列Pの対称化
- ジッター追加 (数値安定性)
- 対角要素のキャップ (状態別上限値)

## ビルド手順

```matlab
cd cpp/build
build_mex()
```

**生成されるMEXファイル**: `cpp/bin/mex_meukf_step_v2.mexw64`

## テスト手順

### 1. 単一実行テスト
```matlab
run_simulation(1, true)  % seed=1, skip data gen
```

**期待結果**:
- NaN発散なし (0回)
- Position RMSE < 1.5m
- Attitude RMSE < 1.0°

### 2. バッチテスト (3回)
```matlab
for i=1:3
    run_simulation(i, false);
end
```

### 3. 性能比較
```matlab
run_batch_10sets()  % 10回実行して統計分析
```

**ベースライン性能** (フィルタリングなし時):
- Position RMSE: 0.75m ± 0.15m
- Attitude RMSE: 0.28-0.64°
- Success rate: 100%

**問題時の性能** (削除後):
- NaN divergence: 500+ resets (Step 15000-20000)
- フィルタ失敗

## 今後の削除手順 (正しい方法)

### ✅ 削除可能なファイル (C++移行済み)

削除するファイルはありません - まずテスト完了を確認する必要があります。

### ⚠️ テスト確認項目

1. **NaN発散チェック**
   ```matlab
   % run_simulation実行後
   grep "Reset - NaN" Results/simulation.log | wc -l
   % 期待値: 0 (ゼロ)
   ```

2. **性能チェック**
   ```matlab
   % batch_10sets実行後
   csvread('Results/batch_10sets_summary.csv')
   % Position RMSE < 1.5m
   % Attitude RMSE < 1.0°
   ```

3. **安定性チェック**
   - 20000ステップ完走
   - 共分散行列Pが正定値
   - 状態変数にInf/NaNなし

### ❌ 削除してはいけないもの (当面)

- **MATLAB側**: ESKF/@ESKF/*.m (統合テスト完了まで保持)
- **C++側**: sensor_filter.hpp, divergence_guard機能 (永続的に必要)

## ステータス

- **ファイル復元**: ✅ 完了
- **C++統合**: ✅ 完了
- **ビルド**: ⚠️ 確認中 (ターミナル出力の問題)
- **テスト実行**: ❌ 未完了 (次のステップ)
- **性能検証**: ❌ 未完了
- **ファイル削除**: ❌ 未実施 (テスト成功後のみ)

## 次のアクション

1. **MEXビルド確認** (手動)
   ```matlab
   cd cpp/build
   build_mex()
   % エラーがないことを確認
   ```

2. **テスト実行** (MATLABコンソールで直接)
   ```matlab
   run_simulation(1, true)
   % NaN警告が出ないことを確認
   ```

3. **性能確認**
   ```matlab
   run_batch_10sets()
   % 結果をCSVで確認
   ```

4. **削除実施** (テスト成功後のみ)
   - C++移行が完全に動作確認できたら
   - KF/Utils/ と Common/Math/QuaternionLib.m を削除
   - ただし、今回は**まだ削除しない**

## 教訓

❌ **誤った手順** (今回の失敗):
1. ファイル削除
2. テスト実行 → NaN発散発見
3. 慌てて復元

✅ **正しい手順**:
1. C++実装を完全に統合
2. ビルド成功を確認
3. テスト実行 → 性能確認
4. バッチテスト → 安定性確認
5. **最後にファイル削除**

## まとめ

- 削除したファイルは全て復元済み ✅
- C++側に完全なセンサーフィルタリングを統合 ✅
- ビルドと動作テストが必要 (次のステップ)
- テスト成功確認後にのみファイル削除を実施

**重要**: 今回はC++統合は完了したが、**テスト確認前には削除しない**。
