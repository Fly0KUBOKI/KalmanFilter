# 行列計算の現状分析レポート

## 📊 調査概要

KalmanFilterプロジェクト内の行列計算実装を横断調査し、重複・不整合・改善点を特定しました。

調査日: 2026年1月6日  
調査範囲: `kalman/cpp/Lib/` 配下の全ライブラリ

---

## 🔍 発見された主要な問題

### 1. **行列計算の実装が複数箇所に散在**

| 実装場所 | 役割 | 問題点 |
|---------|------|--------|
| `Matrix/fixed_matrix.hpp` | テンプレート行列（固定サイズ） | 基本演算のみ実装 |
| `Matrix/fixed_matrix.hpp` | ランタイムサイズ行列（FixedMatrix） | MEX互換性のため別実装 |
| `Common/inc/Math/math_utils.hpp` | 数学ユーティリティ | 行列演算と統計関数が混在 |
| `UKF/inc/ukf_utils.hpp` | UKF専用ユーティリティ | Cholesky分解の3x3特化版 |
| `MEUKF/src/meukf_core.cpp` | MEUKF内部実装 | static関数でCholesky分解を独自実装 |
| `KF/inc/kf_core.hpp` | KFCore実装 | 行列演算を直接for文で記述 |
| `KF/inc/kalman_filter_core.hpp` | KF共通関数 | カルマンフィルタ固有の演算 |

### 2. **Cholesky分解の重複実装（5箇所）**

#### 実装リスト
1. **Matrix::cholesky()** - `fixed_matrix.hpp`（L145-171）
   - テンプレート実装（任意サイズ）
   - 正定値性チェックあり

2. **FixedMatrix::cholesky()** - `fixed_matrix.hpp`（L330-350）
   - ランタイムサイズ版
   - テンプレート版とほぼ同一ロジック

3. **MathUtils::safe_cholesky()** - `math_utils.hpp`（L334-389）
   - FixedMatrix用
   - 正定値でない場合に単位行列へフォールバック
   - 上三角を明示的にゼロクリア

4. **ukf_utils::cholesky3x3()** - `ukf_utils.hpp`（L23-51）
   - 3x3特化版（テンプレート）
   - シンプルなループ実装

5. **meukf::cholesky3x3()** - `meukf_core.cpp`（L252-285）
   - 3x3専用（static関数）
   - ukf_utils版と実装がほぼ同一

#### ロバスト版Cholesky（2箇所）
- **ukf_utils::cholesky3x3_robust()** - 多段フォールバック（対称化→正則化→Cholesky→強正則化→対角近似）
- **meukf::cholesky3x3_robust()** - ukf_utils版と完全に同一アルゴリズム

**重複の影響**:
- メンテナンス負荷増大（修正が5箇所必要）
- アルゴリズムの不整合リスク
- コードサイズの無駄な増加

---

### 3. **3x3行列逆行列の重複実装（2箇所）**

1. **Matrix::inverse()** - Gauss-Jordan消去法（任意サイズ）
2. **MathUtils::invert3x3()** - 解析的逆行列（行列式＋余因子行列）

**問題**:
- 3x3の場合、解析的手法の方が高速だが、使い分けルールが不明確
- inverse() は汎用だがオーバーヘッドあり

---

### 4. **行列演算のインターフェース不統一**

#### パターンA: メンバー関数
```cpp
Matrix<3,3,float> A, B;
Matrix<3,3,float> C = A * B;           // 演算子オーバーロード
Matrix<3,3,float> At = A.transpose();  // メンバー関数
```

#### パターンB: 直接ループ記述（KF/kf_core.hpp）
```cpp
// Predict: P = F*P*F' + Q
cm FP; FP.resize(n_, n_);
for (int i = 0; i < n_; i++) {
    for (int j = 0; j < n_; j++) {
        float sum = 0.0f;
        for (int k = 0; k < n_; k++) {
            sum += F(i, k) * P_(k, j);
        }
        FP(i, j) = sum;
    }
}
cm P_new; P_new.resize(n_, n_);
for (int i = 0; i < n_; i++) {
    for (int j = 0; j < n_; j++) {
        float sum = Q_(i, j);
        for (int k = 0; k < n_; k++) {
            sum += FP(i, k) * F(j, k);  // F(j,k) は転置相当
        }
        P_new(i, j) = sum;
    }
}
```

**問題**:
- 可読性低下（意図が不明瞭）
- 演算子オーバーロードを使えば `P_new = F * P * F.transpose() + Q` で済む
- バグの温床（インデックス誤り）

---

### 5. **カルマンフィルタ固有演算の散在**

#### Innovation計算（3箇所に分散）
1. **KalmanFilterCore::compute_innovation_and_S()** - `kalman_filter_core.hpp`
   - 委譲先: `MathUtils::compute_innovation_and_S()`

2. **MathUtils::compute_innovation_and_S()** - `math_utils.hpp`（動的版）
   - `y = z - h`, `S = H*P*H' + R`

3. **MathUtils::compute_innovation_and_S()** - テンプレート版（固定サイズ）
   - 同一処理だが型が異なる

**問題**:
- 本来KFライブラリに集約すべき
- math_utilsに混在は責務の不明確化

#### カルマンゲイン計算（2箇所）
1. **KalmanFilterCore::compute_kalman_gain()** - テンプレート実装
2. **KFCore::update()** - 直接ループで計算（L157-169）

#### 状態更新（Joseph form）
- **KalmanFilterCore::update_state_covariance()** のみ（集約済み）
- ただし `KFCore::update()` では簡易版 `P = (I-K*H)*P` を使用

---

### 6. **対称化処理のばらつき**

#### 対称化の実装パターン
```cpp
// パターン1: 上三角のみ更新
for (int i = 0; i < n; ++i) {
    for (int j = i+1; j < n; ++j) {
        float avg = (P(i,j) + P(j,i)) * 0.5f;
        P(i,j) = avg;
        P(j,i) = avg;
    }
}

// パターン2: 全体を再計算
P = (P + P.transpose()) * 0.5f;

// パターン3: enforce_symmetry() 関数呼び出し
P = MathUtils::enforce_symmetry(P);
```

**問題**:
- 同じ目的だが実装が統一されていない
- 最適化の余地（演算子オーバーロード vs 直接ループ）

---

### 7. **正定値化・正則化の不統一**

#### 実装箇所
1. **ukf_utils::ensure_positive_definite_3x3()** - 3x3専用
2. **meukf::ensure_positive_definite()** - 3x3専用（static関数）
3. **MathUtils::safe_cholesky()** - Cholesky失敗時に単位行列
4. **sensor_filter::regularize_covariance()** - センサー層での正則化

**問題**:
- 戦略がバラバラ（対角加算 vs 単位行列置換）
- 呼び出しタイミングが不明確

---

### 8. **Mahalanobis距離計算の重複（2実装）**

1. **MathUtils::mahalanobis_distance_squared()** - `math_utils.hpp`（L393-453）
   - Cholesky分解経由
   - フォールバック: 対角最大値で正規化

2. **sensor_filter** 内での独自計算（詳細未確認だが存在の可能性）

---

## 📈 定量的分析

### 重複コード量
| 機能 | 重複箇所数 | 推定行数 | 統合削減可能行数 |
|------|-----------|---------|----------------|
| Cholesky分解 | 5 | 約200行 | 約160行（80%削減） |
| 3x3逆行列 | 2 | 約60行 | 約30行 |
| 対称化処理 | 4 | 約40行 | 約30行 |
| Innovation計算 | 3 | 約90行 | 約60行 |
| 正定値化 | 4 | 約80行 | 約60行 |
| **合計** | **18** | **約470行** | **約340行削減可能** |

### 影響範囲
- **KF/EKF/UKF/ESKF/MEUKF**: 全フィルタが影響
- **メンテナンス工数**: 1つの修正に5箇所の変更が必要（Cholesky分解の場合）

---

## 🏗️ 依存関係マップ

```
┌─────────────────────────────────────────────────┐
│  Matrix/fixed_matrix.hpp                        │
│  - Matrix<R,C,T> (テンプレート)                 │
│  - FixedMatrix (ランタイムサイズ)                │
│  - 基本演算 (+, -, *, transpose, inverse, etc.) │
└───────────────┬─────────────────────────────────┘
                │
    ┌───────────┴───────────────────┬─────────────────┬──────────────┐
    │                               │                 │              │
┌───▼──────────────┐  ┌─────────────▼──────────┐  ┌──▼────────┐  ┌──▼─────────┐
│ Common/Math/     │  │ KF/                    │  │ UKF/      │  │ MEUKF/     │
│ math_utils.hpp   │  │ kalman_filter_core.hpp │  │ ukf_utils │  │ meukf_core │
│ - 統計/座標変換  │  │ - KF固有演算           │  │ - 3x3特化 │  │ - 独自実装 │
│ - Cholesky(重複) │  │ - Innovation/Gain      │  │ - Cholesky│  │ - Cholesky │
│ - 3x3逆行列      │  │ - Joseph form          │  │   (重複)  │  │   (重複)   │
└──────────────────┘  └────────────────────────┘  └───────────┘  └────────────┘
         │                      │                       │               │
         └──────────────────────┴───────────────────────┴───────────────┘
                                      │
                         ┌────────────▼────────────┐
                         │ KF/kf_core.hpp          │
                         │ - 直接ループでの行列演算 │
                         │ - 演算子未使用           │
                         └─────────────────────────┘
```

---

## 🎯 改善の優先順位

### 優先度 HIGH（即座に対応すべき）
1. **Cholesky分解の統一** - 5箇所 → 1箇所（Matrix層）
2. **kf_core.hpp の行列演算リファクタリング** - 演算子オーバーロード活用
3. **Innovation/Gain計算のKFライブラリ集約**

### 優先度 MEDIUM
4. **3x3逆行列の統一** - 解析的手法を優先
5. **対称化処理の統一** - テンプレート関数化
6. **正定値化戦略の標準化**

### 優先度 LOW
7. **Mahalanobis距離のセンサー層との統合**
8. **統計関数のmath_utilsからの分離**

---

## ✅ 推奨される改善アプローチ

### Phase 1: 基本演算の整理（Matrix層）
- Cholesky分解を `Matrix<R,C,T>` に統合
- robust版も含めてテンプレート化
- 3x3専用の最適化版を残す（条件コンパイル）

### Phase 2: KF共通演算の集約（KF層）
- `kalman_filter_core.hpp` に全KF演算を集約
- Innovation/Gain/Joseph formを統一API化

### Phase 3: 既存コードの移行
- `kf_core.hpp` の直接ループを演算子呼び出しに置換
- `meukf_core.cpp` の独自実装を削除
- `ukf_utils.hpp` の重複関数を削除

### Phase 4: ユーティリティの再編成
- math_utils.hpp を分割
  - `matrix_utils.hpp` - 行列専用
  - `statistics.hpp` - 統計関数（既存）
  - `geometry.hpp` - 座標変換

---

## 🚨 リスク評価

### 高リスク領域
- **meukf_core.cpp**: 1346行の巨大ファイル、Cholesky分解を複数箇所で使用
- **kf_core.hpp**: 直接ループでの演算が多数、変更範囲が広い

### 低リスク領域
- **kalman_filter_core.hpp**: すでにテンプレート化済み、影響範囲限定的
- **ukf_utils.hpp**: UKF専用、他への依存が少ない

---

## 📝 備考

- 全ての重複実装は意図的ではなく、段階的開発での自然発生と推測
- MEX互換性（FixedMatrix）の維持が必須条件
- float/double混在問題（Phase 3で部分対応済み）との整合性確保が必要

---

**次ステップ**: `MATRIX_REFACTORING_PLAN.md` で具体的な統一・改善計画を提示します。
