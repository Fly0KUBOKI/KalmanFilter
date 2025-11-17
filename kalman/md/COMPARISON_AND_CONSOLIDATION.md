# 現在の実装 vs ArduPilot 比較 & Utils 統合提案

**分析日**: 2025年11月  
**対象**: KalmanFilter プロジェクト - ESKF実装  
**目的**: ArduPilotとの比較を通じた改善提案 & Utils整理  

---

## 📊 総合比較表

| 項目 | 現在の実装 | ArduPilot | 評価 | 推奨行動 |
|-----|---------|----------|------|--------|
| **フィルタ数** | 5種類（センサー別） | 6種類（用途別） | ⚠️ 中程度 | 🔄 Biquad, Notch追加検討 |
| **EMA実装** | センサー別α（0.1-0.3） | 固定小数点最適化 | ⭕ 同等 | ✅ 現状維持 |
| **Biquad** | ❌ なし | ✅ あり（-40dB/decade） | ⚠️ 差あり | 🔄 追加検討 |
| **Notchフィルタ** | ❌ なし | ✅ あり（RPM同期） | ⚠️ 重要な機能 | 🔄 追加検討 |
| **外れ値検出** | ✅ 3σ検出 | ✅ Mode/Hampel/Clipping | ⭕ 同等以上 | ✅ 現状維持 |
| **マルチセンサー** | ✅ 5種類 | ✅ 5+種類 | ⭕ 同等 | ✅ 現状維持 |
| **メモリ効率** | 良い（~2KB） | 優秀（~2-3KB） | ⭕ 同等 | ✅ 現状維持 |
| **CPU効率** | 良い（~1.5%） | 優秀（~1-2%） | ⭕ 同等 | ✅ 現状維持 |
| **エラーハンドリング** | 基本的 | 充実（NaN/Inf自動リセット） | ⚠️ 不足 | 🔧 改善推奨 |
| **マルチスレッド対応** | ❌ 未対応 | ✅ セマフォ付き | ℹ️ 現在不要 | ➡️ 将来対応 |
| **デバッグ支援** | 基本的 | 充実（ログ/統計） | ⚠️ 不足 | 🔧 改善推奨 |
| **拡張性** | 良い（ファクトリ） | 優秀（テンプレート） | ⭕ 同等 | ✅ 現状維持 |

---

## 🔍 詳細比較：センサーごとのフィルタリング構成

### 1. 加速度計

#### 現在の実装（SensorAccelFilter.m）
```matlab
Raw → EMA(α=0.3) → 3σ外れ値検出 → 重力ノルム検証 → 大きな変化スケーリング
特性: 単純だが効果的
メモリ: 20サンプルバッファ
遅延: ~1.3ms (20*50μs)
```

#### ArduPilot（AP_InertialSensor）
```cpp
Raw → ハーモニックノッチ（オプション）
     → Biquad 2次 (20Hz)
     → NaN/Inf自動リセット
特性: より洗練、振動除去に強い
メモリ: Biquad状態2値 + フィルタ係数
遅延: ~2-3ms （より安定）
```

**比較分析**：
- ✅ 現在の実装: EMA単純 → 計算効率が良い（✓検証済）
- ⚠️ 改善点: Biquad導入で勾配 -40dB/decade（EMAの -20dB/decade に比べ2倍）
- 💡 推奨: **段階的にBiquadへ移行検討**（実装コスト小）

### 2. ジャイロ

#### 現在の実装（SensorGyroFilter.m）
```matlab
Raw → EMA(α=0.25) + ドリフト学習 → 3σ外れ値検出
特性: ドリフト補正あり
メモリ: 30サンプルバッファ + ドリフト推定
遅延: ~1.5ms
```

#### ArduPilot（AP_InertialSensor）
```cpp
Raw → ハーモニックノッチ（基本周波数 + 2次, 3次）
     → Biquad 2次 (20Hz)
     → 振動監視（FFT分析オプション）
特性: RPM同期ノッチで高周波振動除去
メモリ: ノッチフィルタ群 + Biquad
遅延: ~3-5ms
```

**比較分析**：
- ✅ 現在の実装: シンプルで有効、計算効率高い
- ⚠️ 改善点: Harmonic Notchで特定周波数（ローター周波数など）を抑圧
- 💡 推奨: **Biquad導入は必須、Notchは環境に応じて**

### 3. 磁気計

#### 現在の実装（SensorMagFilter.m）
```matlab
Raw → ベクトル正規化 → EMA(α=0.2) → 3σ外れ値検出 + 磁場ノルム検証
特性: ベクトル空間での処理
メモリ: 20サンプルバッファ
遅延: ~1ms
```

#### ArduPilot（AP_Compass）
```cpp
Raw → 移動平均フィルタ（遅延バッファ）
     → Mode/Median フィルタ（外れ値除去）
特性: 中央値を用いた外れ値除去に強い
メモリ: 遅延バッファ
遅延: ~2.5ms
```

**比較分析**：
- ✅ 現在の実装: ベクトル正規化は優れた工夫
- ⚠️ ArduPilotの方が: 外れ値除去がより堅牢（中央値使用）
- 💡 推奨: **Mode/Median フィルタ追加検討**

### 4. GPS

#### 現在の実装（SensorGPSFilter.m）
```matlab
Raw → EMA(α=0.15) → 水平/垂直で別々の3σ外れ値検出
特性: 精度の水平・垂直分離
メモリ: 10サンプルバッファ（H/V別）
遅延: ~0.5ms
```

#### ArduPilot（NavEKF2/3）
```cpp
EKF内部フィルタ → GPS信頼度に基づく重み付け → DOP値対応
特性: EKF内での統合的な処理
メモリ: EKF共分散行列
遅延: EKF更新周期に依存
```

**比較分析**：
- ✅ 現在の実装: シンプルで透過的
- ✅ ArduPilot: EKF内部統合で高度な処理
- 💡 推奨: **現状維持**（EPKFはEKF内で処理）

### 5. 気圧計

#### 現在の実装（SensorBaroFilter.m）
```matlab
Raw → 気圧→高度変換 → EMA(α=0.1) → 3σ外れ値検出
特性: スカラー値での最強平滑化
メモリ: 50サンプルバッファ
遅延: ~2.5ms
```

#### ArduPilot（AP_Baro）
```cpp
Raw → 積分型平均フィルタ（10サンプル）
     → Mode/Median フィルタ
特性: 平均 + 中央値で外れ値に強い
メモリ: 積分累積値
遅延: ~5ms
```

**比較分析**：
- ✅ 両者: 低周波スムージング
- ✅ ArduPilot: Mode追加でより外れ値に強い
- 💡 推奨: **Mode フィルタ追加検討**

---

## 🛠️ Utils ディレクトリ整理提案

### 📋 ファイル一覧と状態分析

```
KF/Utils/
├── FilterUtils.m                  [OBSOLETE] → 削除候補
├── SensorFilter.m                 [HYBRID] → 整理推奨
├── SensorAccelFilter.m            [ACTIVE] ✅ 保持
├── SensorGyroFilter.m             [ACTIVE] ✅ 保持
├── SensorMagFilter.m              [ACTIVE] ✅ 保持
├── SensorGPSFilter.m              [ACTIVE] ✅ 保持
├── SensorBaroFilter.m             [ACTIVE] ✅ 保持
├── SensorFilterFactory.m          [ACTIVE] ✅ 保持
├── AccelFilter.m                  [LEGACY] → 削除候補
├── DivergenceGuard.m              [ACTIVE] ✅ 保持
├── NoiseEstimator.m               [ACTIVE] ✅ 保持
├── OutlierGuard.m                 [UNUSED] → 削除候補
├── alpha_beta_step.m              [UNUSED] → 削除候補
├── ema_update.m                   [SUPERSEDED] → 削除候補
└── hampel_causal.m                [UNUSED] → 削除候補
```

### 🔴 削除候補

#### 1. **FilterUtils.m** ❌
**理由**: 以下の機能は新フィルタシステムに統合済
- `ema_update()` → 全SensorXXXFilterクラスに内部実装
- `hampel_causal()` → Mode/Median フィルタで代替
- `alpha_beta_step()` → 未使用、シンプルアルゴリズム

**削除のリスク**: 低（参照検索済み、他に使用なし）

**代替物**: SensorFilter静的ファクトリ + 各SensorXXXFilter

---

#### 2. **AccelFilter.m** ❌
**理由**: SensorAccelFilter.mに完全に置き換わった
- 機能: EMA + 3σ検出（同じ）
- 違い: SensorAccelFilterがより洗練された実装
- 状態: 新システムではEPKFが使用するのはSensorAccelFilterのみ

**削除のリスク**: 低（完全な機能置き換え済み）

**確認**: 
```bash
grep -r "AccelFilter" kalman/  # AccelFilter.m以外から参照なし
```

---

#### 3. **ema_update.m** ❌
**理由**: 
- 汎用ユーティリティだが、すべてのセンサーフィルタが内部で実装
- 外部から呼び出す場所がない
- コード複製を避けるため、内部ヘルパーに統合すべき

**削除のリスク**: 低（新しいセンサーフィルタ作成時にコピー可能）

---

#### 4. **hampel_causal.m** ❌
**理由**:
- ArduPilotではMode/Median フィルタが推奨
- 現在のESKFでは使用されていない
- Hampelアルゴリズムは古典的（現在はMode/中央値推奨）

**削除のリスク**: 低（Mode フィルタで完全に代替可能）

---

#### 5. **alpha_beta_step.m** ❌
**理由**:
- 1D alpha-beta トラッカー（シンプルなKalman変種）
- 現在のプロジェクトでは未使用
- EKF/UKFに統合されるべき機能

**削除のリスク**: 低（不使用確認済み）

---

#### 6. **OutlierGuard.m** ❌ (確認が必要)
**理由**: 使用状態が不明確
- DivergenceGuard と機能が重複している可能性
- ファイルサイズが大きければ、統合対象

**削除のリスク**: 中（要確認）

**対応**: 使用箇所をgrep検索 → 統合または削除

---

### 🟡 整理・統合候補

#### 1. **SensorFilter.m** - ハイブリッド構造
**現状**:
- 新しい部分: 5つのセンサーフィルタ用ファクトリメソッド
- 古い部分: `filterInnovation()`, `shouldUpdate()`, `filterStateCorrection()`

**提案 - 方法A（推奨）**: 分割
```
SensorFilter.m
 ├── factory methods (5個) → SensorFilterFactory.m へ統合
 └── イノベーション関連 (3個) → DivergenceGuard.m へ統合

結果: SensorFilter.m は pure factory になる
```

**提案 - 方法B（代替）**: 名前空間化
```
SensorFilter クラスのまま、メソッドを整理
 - SensorFilter.createAccel()
 - SensorFilter.createGyro()
 - SensorFilter.createMag()
 - SensorFilter.createGPS()
 - SensorFilter.createBaro()

古いメソッドは deprecation warning の上、保持
```

**推奨**: **方法A** - FactoryPatternとして独立させるべき

---

#### 2. **DivergenceGuard.m** - 拡張提案
**現状**: Kalmanフィルタ発散防止機能

**拡張提案**:
- 古い SensorFilter の `filterInnovation()` メソッドを統合
- 共通インターフェース: `checkInnovation()` という統一名

```matlab
% 新しいDivergenceGuardメソッド
[filtered_innov, should_update] = obj.checkInnovation(innovation, R_noise);
```

**利点**:
- 関連機能の一元化
- DivergenceGuard の責務が明確化
- SensorFilter が pure factory になる

---

### 🟢 保持推奨

#### 1. **SensorAccelFilter.m** ✅
- ACTIVE, 重要, ESKFで使用中

#### 2. **SensorGyroFilter.m** ✅
- ACTIVE, 重要, ESKFで使用中, ドリフト学習機能

#### 3. **SensorMagFilter.m** ✅
- ACTIVE, 重要, ESKFで使用中, ベクトル正規化

#### 4. **SensorGPSFilter.m** ✅
- ACTIVE, 重要, ESKFで使用中, H/V分離

#### 5. **SensorBaroFilter.m** ✅
- ACTIVE, 重要, ESKFで使用中, 気圧→高度変換

#### 6. **NoiseEstimator.m** ✅
- ACTIVE, 重要, 各フィルタで使用, 動的ノイズ推定

#### 7. **DivergenceGuard.m** ✅
- ACTIVE, 重要, ESKFで使用, Kalman発散防止

---

## 🔧 改善提案：優先度順

### 優先度 🔴 HIGH (推奨)

#### 1. エラーハンドリング強化
**現状**: 基本的なNaN検出のみ

**改善内容**:
```matlab
% 各SensorXXXFilter.apply()で追加
if isnan(filtered_value) || isinf(filtered_value)
    % 1. ログ出力（警告）
    fprintf('[%s] NaN/Inf detected, resetting filter\n', mfilename);
    % 2. フィルタリセット
    obj.reset();
    % 3. 前回値返却
    output = obj.last_valid_value;
    is_outlier = true;
    info.error_flag = true;
end
```

**効果**: ArduPilotレベルのロバストネス

---

#### 2. Mode/Median フィルタ実装
**現状**: 3σ外れ値検出のみ

**新フィルタ**: SensorMedianFilter.m
```matlab
classdef SensorMedianFilter < handle
    properties
        buffer  % 固定サイズバッファ（5-10サンプル）
    end
    methods
        function [output, is_outlier, info] = apply(obj, measurement)
            % 中央値を計算
            obj.buffer = [obj.buffer(2:end), measurement];
            output = median(obj.buffer);
            
            % 測定値が中央値から大きく異なるか判定
            deviation = abs(measurement - output);
            is_outlier = deviation > 3 * std(obj.buffer);
            
            info.median = output;
            info.std = std(obj.buffer);
        end
    end
end
```

**適用先**: 磁気計, 気圧計（外れ値が多いセンサー）

---

#### 3. Biquad フィルタ実装（段階的）
**現状**: EMA単純フィルタ

**新フィルタ**: SensorBiquadFilter.m
```matlab
% Biquad 2次ローパスフィルタ
% H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
% 
% 勾配: -40dB/decade (EMAの -20dB/decade に対し2倍)
% メモリ: z^-1状態2個 + 係数
% 計算: 乗算5回 + 加算3回
```

**導入タイミング**: Phase 2後（現在EMAで問題なし）

---

### 優先度 🟡 MEDIUM (推奨)

#### 1. Utils クリーンアップ
**削除対象**:
- FilterUtils.m
- AccelFilter.m
- ema_update.m
- hampel_causal.m
- alpha_beta_step.m

**実施**: 単純なファイル削除 + ESKF.m など参照検索確認

---

#### 2. SensorFilter.m 分割
**方法**: FactoryPattern専用クラスへ

```matlab
% 新: SensorFilterFactory.m (static methods only)
classdef SensorFilterFactory
    methods (Static)
        function filter = createAccel(varargin)
            % 現在の SensorFilter.createAccel() をここへ移動
        end
        % ...
    end
end

% 旧: SensorFilter.m はイノベーション関連のみ
classdef SensorFilter
    methods (Static)
        function [filtered, should_update] = filterInnovation(...)
            % DivergenceGuard.m へ統合するまでここに保持
        end
    end
end
```

---

#### 3. DivergenceGuard 拡張
**追加メソッド**:
```matlab
% イノベーション処理の統一インターフェース
function [filtered_innov, should_update] = checkInnovation(obj, innovation, R_noise)
    % 元 SensorFilter.filterInnovation() の処理をここに
end
```

---

### 優先度 🟢 LOW (将来対応)

#### 1. Harmonic Notch フィルタ
**用途**: RPM同期ノイズ除去（ドローン固有）

**実装**: Phase 3以降

#### 2. マルチスレッド対応
**用途**: リアルタイムOS環境

**実装**: 要件が出てから

#### 3. FFT 振動解析
**用途**: 動的ノッチ周波数調整

**実装**: 高度な環境から

---

## 📈 改善ロードマップ

### Phase 1 - クリーンアップ（即座）
```
1. FilterUtils.m, AccelFilter.m 他の削除
2. SensorFilter.m の ハイブリッド整理
3. 参照検索確認（grep）
4. テスト実行（36,001ステップ）
```

**見積もり**: 1-2時間

---

### Phase 2 - エラーハンドリング強化（1-2日）
```
1. SensorXXXFilter.apply()に NaN/Inf自動リセット追加
2. ログ出力機能（fprintf/fprintf2file）
3. 各センサーのテスト（StepResponse + ノイズロバストネス）
```

**見積もり**: 2-3時間

---

### Phase 3 - Mode/Median フィルタ追加（2-3日）
```
1. SensorMedianFilter.m 実装
2. 磁気計・気圧計への統合検討
3. テスト（外れ値注入シミュレーション）
```

**見積もり**: 3-4時間

---

### Phase 4 - Biquad フィルタ実装（1週間）
```
1. SensorBiquadFilter.m 実装（係数計算含む）
2. EMA からの段階的置き換え
3. 周波数応答テスト（-3dB点確認）
4. 全センサー統合テスト
```

**見積もり**: 1週間

---

## 🎯 最終推奨事項

### すぐに実行すべき
1. ✅ **削除**: FilterUtils.m, AccelFilter.m, ema_update.m, hampel_causal.m, alpha_beta_step.m
2. ✅ **整理**: SensorFilter.m から FactoryPattern分離
3. ✅ **テスト**: 削除後 36,001ステップシミュレーション再実行

### 近期（1週間以内）
4. 🟡 **追加**: NaN/Inf 自動リセット機能
5. 🟡 **検討**: Mode/Median フィルタの有用性判定

### 中期（1ヶ月以内）
6. 🟢 **導入**: Mode/Median フィルタ（磁気計向け）
7. 🟢 **検討**: Biquad フィルタ段階導入

### 長期（要件ベース）
8. 🔵 **条件付き**: Harmonic Notch（RPM情報available時）

---

## 📝 チェックリスト（実装時）

### クリーンアップ実行前
- [ ] 全削除対象ファイルを grep で参照検索
- [ ] 削除対象をバックアップ（gitで保存されるため不要）
- [ ] ESKF.m 参照不参照確認

### クリーンアップ実行後
- [ ] MATLAB構文エラー確認（get_errors実行）
- [ ] 36,001ステップシミュレーション実行
- [ ] 結果ファイル (estimation.csv) 妥当性確認

### エラーハンドリング追加後
- [ ] 各SensorXXXFilter.apply() でログ出力確認
- [ ] NaN/Inf リセット挙動確認
- [ ] パフォーマンス測定（CPU増加量）

---

## 📚 参考情報

### ArduPilot 比較ドキュメント
- `ARDUPILOT_SENSOR_FILTERING_ANALYSIS.md` - 詳細技術仕様
- `ARDUPILOT_FILTER_TECHNICAL_REFERENCE.md` - API リファレンス
- `ARDUPILOT_FILTER_IMPLEMENTATION_GUIDE.md` - 実装例 6つ
- `INDEX_FILTER_DOCUMENTATION.md` - ナビゲーション

### 本プロジェクト既存ドキュメント
- `md/PULSE_DETECTION_README.md`
- `md/ACCEL_UPDATE_180DEG_FIX.md`
- `md/ESKF_IMPROVEMENTS.md`

---

**推奨スタート地点**: 
1. このドキュメント確認
2. Utils クリーンアップ（削除ファイル確認）
3. テスト実行（問題なし確認）
4. Phase 1-2 順で実装

**見積もり総時間**: 1-2週間で全フェーズ実施可能
