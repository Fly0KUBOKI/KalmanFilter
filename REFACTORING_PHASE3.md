# Phase 3: 冗長コード・重複の廃止

**目標**: 重複クラス定義、重複ファイルを統合・削除  
**所要時間**: 2時間  
**リスク**: 高（依存関係の確認が必須）  
**前提条件**: Phase 2 完了

---

## 1. 問題の概要

### 1.1 重複クラス定義

| クラス | 定義場所 | 状態 |
|-------|---------|------|
| NoiseEstimator | robust_statistics.hpp | ✅ 正式版 |
| NoiseEstimator | validation.hpp | ❌ 重複（削除対象） |
| OutlierDetector | outlier_detector.hpp | ✅ 正式版 |
| OutlierDetector | validation.hpp | ❌ 重複（削除対象） |
| DivergenceGuard | robust_statistics.hpp | ✅ 正式版 |
| DivergenceGuard | validation.hpp | ❌ 重複（削除対象） |

### 1.2 重複ディレクトリ

```
MEX/
├── Impl/                    ← 正式版
│   ├── mex_eskf_common.hpp
│   ├── mex_run_eskf_impl.hpp
│   ├── mex_run_eskf_sensor_updates.hpp
│   ├── mex_run_eskf.cpp
│   └── ...
└── Inc/                     ← 完全重複（削除対象）
    ├── mex_eskf_common.hpp
    ├── mex_run_eskf_impl.hpp
    ├── mex_run_eskf_sensor_updates.hpp
    └── ...
```

---

## 2. validation.hpp の修正

### 2.1 現状

`Lib/Common/inc/validation.hpp` (254行) に以下の重複定義がある:
- OutlierDetector (行30-100)
- NoiseEstimator (行101-180)
- DivergenceGuard (行181-254)

### 2.2 修正方針

1. 重複クラス定義を削除
2. 正式版をincludeに変更
3. 本来のバリデーション機能のみ残す

### 2.3 修正後の validation.hpp

```cpp
#pragma once

#ifndef COMMON_VALIDATION_HPP
#define COMMON_VALIDATION_HPP

#include "../../Matrix/fixed_matrix.hpp"
#include "Sensor/outlier_detector.hpp"
#include "Sensor/robust_statistics.hpp"
#include <cmath>
#include <cfloat>

namespace common {
namespace validation {

using cm = cmath_fx::FixedMatrix;

// 正式版クラスのエイリアス（後方互換性のため）
using OutlierDetector = common::sensor::OutlierDetector;
using NoiseEstimator = common::sensor::NoiseEstimator;
using DivergenceGuard = common::sensor::DivergenceGuard;

// ========== 共分散バリデーション ==========
// 共分散行列の対称性・正定値性を確認
template<typename T>
bool validate_covariance(const cmath_fx::Matrix<15,15,T>& P, T tolerance = 1e-6) {
    // 対称性チェック
    for(int i = 0; i < 15; ++i) {
        for(int j = i+1; j < 15; ++j) {
            if(std::fabs(P(i,j) - P(j,i)) > tolerance) {
                return false;
            }
        }
    }
    // 対角成分が正か
    for(int i = 0; i < 15; ++i) {
        if(P(i,i) <= 0) {
            return false;
        }
    }
    return true;
}

// ========== 状態バリデーション ==========
// 四元数の正規化チェック
template<typename T>
bool validate_quaternion(const T q[4], T tolerance = 1e-4) {
    T norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    return std::fabs(norm - 1.0) < tolerance;
}

// 数値の有限性チェック
template<typename T>
bool is_finite(const T* arr, int n) {
    for(int i = 0; i < n; ++i) {
        if(!std::isfinite(arr[i])) return false;
    }
    return true;
}

} // namespace validation
} // namespace common

#endif
```

### 2.4 削除される行数

- 削除: 約200行（重複クラス定義）
- 追加: 約50行（バリデーション関数）
- 結果: 254行 → 約60行

---

## 3. MEX/Inc/ ディレクトリの削除

### 3.1 削除前確認

```bash
# MEX/Inc/ をインクルードしている箇所を検索
grep -r "MEX/Inc/" kalman/cpp --include="*.hpp" --include="*.cpp"

# mex_run_eskf.cpp のincludeパスを確認
head -30 kalman\cpp\MEX\Impl\mex_run_eskf.cpp
```

**確認事項**:
- `mex_run_eskf.cpp` が `Inc/` と `Impl/` どちらを参照しているか
- 両方参照している場合、`Impl/` に統一

### 3.2 mex_run_eskf.cpp のinclude修正（必要に応じて）

```cpp
// 変更前（もし Inc/ を参照している場合）
#include "../Inc/mex_eskf_common.hpp"
#include "../Inc/mex_run_eskf_impl.hpp"

// 変更後（Impl/ に統一）
#include "mex_eskf_common.hpp"
#include "mex_run_eskf_impl.hpp"
```

### 3.3 削除実行

```bash
# フォルダ全体を削除
rmdir /s /q kalman\cpp\MEX\Inc
```

### 3.4 削除されるファイル

| ファイル | 行数 |
|---------|------|
| mex_eskf_common.hpp | 約200行 |
| mex_eskf_init.hpp | 約150行 |
| mex_run_eskf_impl.hpp | 約300行 |
| mex_run_eskf_sensor_updates.hpp | 約200行 |
| mex_type_conversion.hpp | 約150行 |
| sensor_preprocessor.hpp | 約100行 |
| **合計** | **約1,100行** |

---

## 4. eskf_runner.hpp の重複コード確認

### 4.1 確認対象

`Lib/ESKF/inc/eskf_runner.hpp` に以下の可能性:
- 他ファイルと重複するユーティリティ関数
- インライン化すべき小関数

### 4.2 確認コマンド

```bash
# eskf_runner.hpp の関数一覧
grep -n "inline\|void\|bool\|float\|double" kalman\cpp\Lib\ESKF\inc\eskf_runner.hpp | head -50
```

### 4.3 統合候補

以下の関数が複数ファイルに存在する場合、正式版1箇所に統合:
- `normalize_quaternion()` → `cquat::normalize_quat<T>()` に統一
- `symmetrize_covariance()` → `filter_mgmt.hpp` に統一
- `clamp()` → `common::math` に統一

---

## 5. 未使用ファイルの削除

### 5.1 確認対象ディレクトリ

| ディレクトリ | 状態 | 対応 |
|-------------|------|------|
| Lib/KF/ | 使用状況不明 | 確認後削除可能 |
| Lib/EKF/ | 使用状況不明 | 確認後削除可能 |
| Lib/UKF/ | 未完成 | 確認後削除可能 |

### 5.2 使用確認コマンド

```bash
# KF/ の使用確認
grep -r "KF/" kalman/cpp --include="*.hpp" --include="*.cpp" | grep -v "KF/inc"
grep -r "kf_operations" kalman/cpp --include="*.hpp" --include="*.cpp"

# EKF/ の使用確認
grep -r "EKF/" kalman/cpp --include="*.hpp" --include="*.cpp"
grep -r "ekf_core" kalman/cpp --include="*.hpp" --include="*.cpp"

# UKF/ の使用確認
grep -r "UKF/" kalman/cpp --include="*.hpp" --include="*.cpp"
grep -r "ukf_core" kalman/cpp --include="*.hpp" --include="*.cpp"
```

### 5.3 判断基準

- **使用されている**: 残す
- **ESKF/MEUKFからのみ参照**: kf_operations.hpp のみ残し、他は削除
- **未使用**: 削除

---

## 6. 詳細な実施手順

### Step 1: 依存関係の確認

```bash
# validation.hpp の使用箇所
grep -rn "validation.hpp" kalman/cpp --include="*.hpp" --include="*.cpp"

# 各重複クラスの使用箇所
grep -rn "OutlierDetector" kalman/cpp --include="*.hpp" --include="*.cpp"
grep -rn "NoiseEstimator" kalman/cpp --include="*.hpp" --include="*.cpp"
grep -rn "DivergenceGuard" kalman/cpp --include="*.hpp" --include="*.cpp"
```

### Step 2: validation.hpp の修正

1. バックアップ作成
2. 重複クラス定義を削除
3. include文を追加
4. using宣言で後方互換性を確保

### Step 3: MEX/Inc/ 削除

1. include パス確認
2. mex_run_eskf.cpp の修正（必要な場合）
3. Inc/ ディレクトリ削除

### Step 4: ビルド確認

```matlab
cd kalman/cpp/build
clear mex
build_mex()
```

### Step 5: 回帰テスト

```matlab
clear mex
cd ../..
run_batch_10sets()
```

---

## 7. 修正前後の比較

### 7.1 validation.hpp

```diff
 #pragma once
 
 #ifndef COMMON_VALIDATION_HPP
 #define COMMON_VALIDATION_HPP
 
 #include "../../Matrix/fixed_matrix.hpp"
+#include "Sensor/outlier_detector.hpp"
+#include "Sensor/robust_statistics.hpp"
 #include <cmath>
 #include <cfloat>
 
 namespace common {
 namespace validation {
 
 using cm = cmath_fx::FixedMatrix;
 
-// ========== 外れ値検出器 ==========
-class OutlierDetector {
-    // 約70行の定義を削除
-};
+// 正式版クラスのエイリアス
+using OutlierDetector = common::sensor::OutlierDetector;
+using NoiseEstimator = common::sensor::NoiseEstimator;
+using DivergenceGuard = common::sensor::DivergenceGuard;
-
-// ========== ノイズ推定器 ==========
-class NoiseEstimator {
-    // 約80行の定義を削除
-};
-
-// ========== 発散防止 ==========
-class DivergenceGuard {
-    // 約70行の定義を削除
-};
 
+// バリデーション関数のみ残す
+template<typename T>
+bool validate_covariance(const cmath_fx::Matrix<15,15,T>& P, T tolerance = 1e-6);
+
+template<typename T>
+bool validate_quaternion(const T q[4], T tolerance = 1e-4);
+
+template<typename T>
+bool is_finite(const T* arr, int n);
 
 } // namespace validation
 } // namespace common
 
 #endif
```

---

## 8. 完了確認チェックリスト

- [x] validation.hpp から重複クラス定義を削除
- [x] validation.hpp にincludeとusingエイリアスを追加
- [x] MEX/Inc/ ディレクトリを削除
- [x] mex_run_eskf.cpp のincludeパスを確認・修正
- [x] 未使用のKF/EKF/UKFファイルを確認
- [x] `build_mex()` 成功
- [x] `run_batch_10sets()` 10/10 PASS
- [x] Git commit 完了

---

## 9. 削減効果

| 項目 | Before | After | 削減 |
|-----|--------|-------|------|
| validation.hpp | 254行 | 60行 | 76% |
| MEX/Inc/ | 1,100行 | 0行 | 100% |
| **合計** | **1,354行** | **60行** | **96%** |

---

## 10. 次のPhaseへの移行条件

- [x] 全重複クラス定義が統合済み
- [x] 重複ディレクトリ（MEX/Inc/）が削除済み
- [x] ビルド成功
- [x] 回帰テスト合格

**次のPhase**: Phase 4 - クラス設計の最適化

---

## 11. 【重大インシデント報告】Phase3 実装失敗と修正（2026-01-11）

### 11.1 発生事象

**日時**: 2026年1月11日 21:46 - 21:59  
**影響範囲**: 全推定処理が完全停止  
**症状**:
- run_batch_10sets(): 10/10 FAILED
- Position RMSE: 17～68m（正常値0.32m）
- Attitude RMSE: Roll/Pitch/Yaw 100～106 deg（正常値0.3/0.3/0.8 deg）
- Gyro bias (final): [0.0000, 0.0000, 0.0000]（正常値は非ゼロ）
- Max Innovation: 0.0000（センサー更新が一切行われていない）
- 実行時間: 0.30～0.34秒/run（正常値2.1秒）

### 11.2 根本原因

**Phase3のコミット 4104410 で重大な実装ミス**:

1. **削除された重要ファイル**:
   - `MEX/Inc/mex_run_eskf_sensor_updates.hpp` (756行)
   - この中に **MEUKF更新経路の本体** (`handle_sensor_update_internal`) が含まれていた

2. **誤った新実装**:
   ```cpp
   // 旧版（正常動作、756行）
   inline void call_sensor_update(...) {
       // 1. センサー前処理
       // 2. handle_sensor_update_internal 呼び出し
       //    → MEUKF更新を実行
       //    → 状態（p,v,q,ba,bg,P）を書き戻し
   }
   
   // 新版（Phase3で作成、40行）← 完全に誤り
   inline void call_sensor_update(...) {
       // 単に eskf::update_accel_sensor() を呼ぶだけ
       // 状態書き戻しなし
       // MEUKF経路なし
   }
   ```

3. **失われた処理**:
   - `handle_sensor_update_internal()`: MEUKFベースのセンサー更新
   - `mex_meukf_step_v2` 相当の処理経路
   - 状態の書き戻し（`s->p`, `s->v`, `s->q`, `s->ba`, `s->bg`, `s->P`）

### 11.3 影響分析

| 項目 | 正常時 | Phase3誤実装時 | 差異 |
|------|--------|---------------|------|
| Position RMSE | 0.32m | 17～68m | **53～212倍悪化** |
| Roll/Pitch RMSE | 0.3/0.3 deg | 101～106 / 32～41 deg | **300～350倍悪化** |
| Yaw RMSE | 0.8 deg | 74～107 deg | **92～134倍悪化** |
| Gyro bias | 非ゼロ | [0,0,0] | **推定停止** |
| Max Innovation | 正常値 | 0.0000 | **更新なし** |
| 実行時間 | 2.1秒/run | 0.3秒/run | **7倍高速化（異常）** |

**実行時間の異常な短縮の意味**:
- センサー更新処理（MEUKF計算）がスキップされていた
- 単なる予測ステップのみ実行
- これは「最適化」ではなく「処理の欠落」

### 11.4 修正手順

#### Step 1: Git履歴調査
```bash
git log --oneline -15
git show b99c8b4:kalman/cpp/MEX/Inc/mex_run_eskf_sensor_updates.hpp > /tmp/old_sensor_updates.hpp
wc -l /tmp/old_sensor_updates.hpp  # 756行
```

#### Step 2: 旧版の完全復元
```bash
cp /tmp/old_sensor_updates.hpp kalman/cpp/MEX/Impl/mex_run_eskf_sensor_updates.hpp
# ヘッダーガード名を _IMPL サフィックスに修正
```

#### Step 3: 型の不一致修正
```cpp
// mex_run_eskf_impl.hpp で、do_step() の呼び出しを修正
// 旧版は const double* を期待
call_sensor_update(s, "accel", a_d, 3, k);  // a_f ではなく a_d
call_sensor_update(s, "mag", m_d, 3, k);    // m_f ではなく m_d
```

#### Step 4: ビルドと検証
```matlab
% MATLAB終了（MEXロック解除）
taskkill /F /IM MATLAB.exe

% 再ビルド
cd kalman/cpp/build
matlab -batch "build_mex()"
% → OK (1.1 MB)

% 回帰テスト
cd ../..
matlab -batch "clear mex; run_batch_10sets()"
% → 10/10 PASS
```

#### Step 5: 結果確認
```
成功: 10/10 (100.0%)
Position RMSE: Mean=0.3215m, Max=0.3390m
Roll/Pitch/Yaw RMSE: 0.31/0.31/0.79 deg
Gyro bias: 正常な非ゼロ値
実行時間: 2.1秒/run（正常範囲）
```

### 11.5 再発防止策

#### 1. **大規模削除前の機能テスト必須化**
```markdown
【ルール】756行以上のファイル削除時は、削除前後で回帰テストを実行
- 削除前: run_batch_10sets() → 結果を記録
- 削除後: run_batch_10sets() → 差分確認
- 差異がある場合、削除内容を再検討
```

#### 2. **ファイル削除のレビュー基準**
```markdown
以下を満たさない限り、大規模ファイル（500行以上）は削除禁止:
1. 完全な重複であることを grep で確認
2. 他ファイルで同等機能が提供されていることを grep で確認
3. 依存関係がないことを grep で確認
4. 削除後のビルドが成功
5. 削除後の回帰テストが全通過
```

#### 3. **センサー更新経路の監視**
```markdown
以下の指標が異常値を示した場合、即座に前のコミットに戻す:
- Max Innovation = 0.0000（全ステップ）
- Gyro bias = [0, 0, 0]
- 実行時間が50%以上短縮
- Position RMSE > 10m
- Attitude RMSE > 10 deg
```

#### 4. **Git コミット前のチェックリスト**
```markdown
□ ビルド成功
□ run_batch_10sets() 実行
□ 10/10 PASS確認
□ Position RMSE < 1.0m
□ Attitude RMSE < 5.0 deg
□ Gyro bias ≠ [0,0,0]
□ Max Innovation > 0（少なくとも一部ステップで）
□ 実行時間が前回の±20%以内
```

#### 5. **重複削除の正しいプロセス**
```markdown
Phase3で行うべきだった手順:
1. MEX/Inc/ と MEX/Impl/ の完全な diff 確認
2. 756行のファイル内容を行単位で比較
3. handle_sensor_update_internal の存在を確認
4. この関数が MEUKF更新の本体であることを理解
5. Impl/ 版に存在しない場合、Inc/ から移植
6. 移植後にビルド＆テスト
7. 全通過後に Inc/ 削除
```

### 11.6 教訓

1. **「重複」の定義を厳密に**:
   - ファイル名が同じ ≠ 内容が同じ
   - 必ず diff で完全一致を確認

2. **実行時間の異常な短縮は警告信号**:
   - 処理が高速化 → 正常
   - 処理が7倍高速化 → **処理の欠落を疑う**

3. **大規模削除は段階的に**:
   - 1ファイルずつ削除
   - 各削除後に回帰テスト
   - 問題が出た時点で特定可能

4. **センサー更新経路の重要性**:
   - ESKF推定の心臓部
   - この経路が失われると推定は完全停止
   - Max Innovation = 0 は「センサー更新なし」の明確な兆候

### 11.7 修正コミット

```
commit 5e5d6fa
Author: Fly0KUBOKI <takuteru0225@gmail.com>
Date:   Sat Jan 11 21:59:56 2026 +0000

    fix(phase3): restore full sensor_updates implementation (756 lines)
    
    Phase3で削除したMEUKF更新経路（handle_sensor_update_internal）を復元。
    誤って簡易ラッパー（40行）に置き換えてしまい、推定が完全に失敗していた。
```

### 11.8 今後のPhase実施時の注意

Phase 4以降で同様の問題を防ぐため:
- [ ] 大規模削除前に必ず diff 確認
- [ ] 削除対象ファイルの全関数をリスト化
- [ ] 各関数が他のどこかに存在することを grep で確認
- [ ] 削除後、即座に回帰テスト実行
- [ ] 異常値検出時は即座にロールバック

---

**Phase 3 完了条件（修正版）**:
- [x] 全重複クラス定義が統合済み
- [x] 重複ディレクトリ（MEX/Inc/）が削除済み
- [x] **センサー更新経路（handle_sensor_update_internal）が復元済み**
- [x] ビルド成功
- [x] 回帰テスト合格（10/10 PASS, Position RMSE 0.32m）
- [x] インシデント報告書作成完了

**次のPhase**: Phase 4 - クラス設計の最適化
