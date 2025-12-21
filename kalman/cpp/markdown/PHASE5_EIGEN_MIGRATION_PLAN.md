# PHASE 5: Eigen 廃止と自作行列ライブラリ完成

**作成日**: 2025年12月21日  
**ステータス**: 📋 計画フェーズ  
**目的**: Phase 5 へ進む前に Eigen を完全に廃止し、自作ライブラリ (`cmath_fx`) への統一移行

---

## 📊 現状分析

### 1. Eigen の使用状況

#### 使用ファイル（5ファイル）

| ファイル | 用途 | 影響度 |
|---------|------|--------|
| [kalman/cpp/UKF/Core/ukf_sigma_points.hpp](../UKF/Core/ukf_sigma_points.hpp) | Cholesky分解、固有値分解 | **高** |
| [kalman/cpp/UKF/Core/ukf_sigma_points.cpp](../UKF/Core/ukf_sigma_points.cpp) | 実装（分解処理） | **高** |
| [kalman/cpp/include/UKF/ukf_sigma_points.hpp](../include/UKF/ukf_sigma_points.hpp) | インターフェース | **高** |
| [kalman/cpp/src/UKF/ukf_sigma_points.cpp](../src/UKF/ukf_sigma_points.cpp) | 実装複製 | **中** |
| [kalman/cpp/include/EKF/ekf.hpp](../include/EKF/ekf.hpp) | EKF（未使用） | **低** |
| [kalman/cpp/EKF/ekf.hpp](../EKF/ekf.hpp) | EKF 実装（未使用） | **低** |

**Eigen 依存関数**:
- `Eigen::LLT` — Cholesky 分解（UKF シグマポイント計算）
- `Eigen::SelfAdjointEigenSolver` — 固有値分解（正定値性保証）
- `Eigen::MatrixXf`, `Eigen::VectorXf` — 動的行列

#### インクルードパターン
```cpp
#include <Eigen/Dense>
using Matrix = Eigen::MatrixXf;
using Vector = Eigen::VectorXf;
```

---

### 2. 自作ライブラリ（`cmath_fx`）の現状

#### 実装済みコンポーネント

| コンポーネント | 種類 | ファイル | ステータス |
|---|---|---|---|
| **固定サイズ行列** | テンプレート | [fixed_matrix.hpp](../Common/Math/fixed_matrix.hpp) | ✅ 完成 |
| **動的サイズ行列** | ランタイム | [fixed_matrix.hpp](../Common/Math/fixed_matrix.hpp) (`FixedMatrix` struct) | ✅ 完成 |
| **基本演算** | `+`, `-`, `*`, `transpose`, `inverse` | [fixed_matrix.hpp](../Common/Math/fixed_matrix.hpp) | ✅ 完成 |
| **クォータニオン演算** | 姿勢計算 | [quaternion.hpp](../Common/Math/quaternion.hpp), [quaternion_compute.hpp](../Common/Math/quaternion_compute.hpp) | ✅ 完成 |
| **回転行列演算** | 座標変換 | [rotation_compute.hpp](../Common/Math/rotation_compute.hpp) | ✅ 完成 |
| **Math ユーティリティ** | 数値計算補助 | [math_utils.hpp](../Common/Math/math_utils.hpp) | ✅ 完成 |

#### 未実装・不足機能

| 機能 | 必要性 | 代替案 |
|---|---|---|
| **Cholesky 分解** | **高**（UKF） | 自作実装必須 |
| **固有値分解** | **中**（UKF正定値性） | 自作実装必須 |
| **動的メモリ行列** | **中**（デバッグ用） | `FixedMatrix` で十分 |
| **BLAS/LAPACK** | 低 | 不要（小規模行列） |

---

### 3. 利用状況の把握

#### `cmath_fx` 導入済みコンポーネント

```
✅ MEUKF:        cmath_fx::Vector<3,4,15>, cmath_fx::Matrix<3,3,15,15>
✅ ESKF:         cmath_fx::Vector<3,4,15>, cmath_fx::Matrix<3,3,15,15>
✅ MEX 全体:     cmath_fx::FixedMatrix（MATLAB I/F）
✅ UKF Core:     cmath_fx::Vector<N,2N+1>, cmath_fx::Matrix<N,N,NxK> 使用中
```

#### 残存する Eigen 依存

```
❌ UKF Sigma Points: Eigen::LLT, Eigen::SelfAdjointEigenSolver
```

---

## 🎯 移行計画（5フェーズ）

### Phase 5.1: Cholesky 分解の自作実装

**目標**: `FixedMatrix::cholesky()` を実装  
**期間**: 1-2日  
**ファイル**: [kalman/cpp/Common/Math/fixed_matrix.hpp](../Common/Math/fixed_matrix.hpp)

#### 実装内容

```cpp
// 目標: FixedMatrix に追加
namespace cmath_fx {
struct FixedMatrix {
    // Cholesky分解: A = L * L^T（Lは下三角行列）
    // 入力: A (対称正定値行列)
    // 出力: L（下三角行列）
    // 戻り値: 成功時true, 失敗時false
    bool cholesky(FixedMatrix& L) const;
    
    // ヘルパー: Cholesky分解後の sqrt
    bool cholesky_sqrt(FixedMatrix& sqrt_P) const;
};
}
```

#### 実装アルゴリズム（Doolittle 法）

```
1. 正定値性チェック: A[0,0] > 0
2. for k = 0 to n-1:
     L[k,k] = sqrt(A[k,k] - sum(L[k,j]^2, j=0..k-1))
     if L[k,k] <= 0: return false  // 正定値でない
3. for i = k+1 to n-1:
     L[i,k] = (A[i,k] - sum(L[i,j]*L[k,j], j=0..k-1)) / L[k,k]
```

#### テスト

```matlab
% kalman/cpp/MEUKF/test_cholesky.m (新規作成)
P = [4, 2; 2, 3];           % 対称正定値
L_expected = [2, 0; 1, sqrt(2)];
L_result = mex_common_lib('cholesky', P);
assert(norm(L_result*L_result' - P) < 1e-6, 'Cholesky failed');
```

---

### Phase 5.2: 固有値分解の簡易実装

**目標**: `FixedMatrix::eigen_decompose()` を実装  
**期間**: 2-3日  
**ファイル**: [kalman/cpp/Common/Math/fixed_matrix.hpp](../Common/Math/fixed_matrix.hpp)

#### 実装内容

```cpp
// 目標: FixedMatrix に追加
namespace cmath_fx {
struct FixedMatrix {
    // 対称行列の固有値分解（累乗法 + QR 法）
    // 入力: A (対称行列)
    // 出力: eigenvalues（対角要素）, eigenvectors（列がベクトル）
    // 注: 小規模行列（n <= 15）専用
    bool eigen_decompose(
        FixedMatrix& eigenvalues,   // n x 1
        FixedMatrix& eigenvectors   // n x n
    ) const;
};
}
```

#### 実装アルゴリズム（簡易版）

**オプション A: 累乗法（最大固有値のみ）**
- 長所: シンプル、デバッグ容易
- 短所: 全固有値が必要な場合は不向き

**オプション B: QR 反復法**
- 長所: 全固有値・固有ベクトル計算可
- 短所: 実装複雑

**推奨**: **オプション B（QR 法）** — UKF では複数固有値が必要

#### QR 法の実装ステップ

```
1. Hessenberg 行列に変換（オプション、小規模は省略）
2. QR 反復:
   for iter = 0 to max_iter:
       A = QR (QR分解)
       A = R*Q (再構成)
       if 収束: break
3. 対角線要素を固有値として取得
4. 固有ベクトルは各反復後の Q を累積
```

#### テスト

```matlab
% kalman/cpp/MEUKF/test_eigen.m (新規作成)
A = [4, 1; 1, 3];                    % 対称行列
[eigenvalues, eigenvectors] = mex_common_lib('eigen_decompose', A);
% 検証: A * v = lambda * v
assert(norm(A * eigenvectors(:,1) - eigenvalues(1,1) * eigenvectors(:,1)) < 1e-5);
```

---

### Phase 5.3: UKF Sigma Points の移行

**目標**: Eigen 依存を廃止し、`cmath_fx` のみで実装  
**期間**: 1-2日  
**ファイル**:
- [kalman/cpp/UKF/Core/ukf_sigma_points.hpp](../UKF/Core/ukf_sigma_points.hpp)
- [kalman/cpp/UKF/Core/ukf_sigma_points.cpp](../UKF/Core/ukf_sigma_points.cpp)

#### 修正箇所

```diff
// Before
#include <Eigen/Dense>
using Matrix = Eigen::MatrixXf;
using Vector = Eigen::VectorXf;

// Sigma Points 生成
Eigen::LLT<Matrix> llt(P_scaled);
if (llt.info() != Eigen::Success) { /* error */ }
sqrtP = llt.matrixL();

// After
#include "../Common/Math/fixed_matrix.hpp"
using Matrix = cmath_fx::FixedMatrix;  // または cmath_fx::Matrix<N,N>

// Sigma Points 生成
if (!P_scaled.cholesky_sqrt(sqrtP)) { /* error */ }
```

#### 実装パターン

```cpp
// kalman/cpp/UKF/Core/ukf_sigma_points.cpp
void UKFSigmaPoints::generate(
    const cmath_fx::FixedMatrix& P,
    cmath_fx::FixedMatrix& sigma_pts,
    cmath_fx::FixedMatrix& wm,
    cmath_fx::FixedMatrix& wc
) {
    int n = P.rows;
    float gamma = sqrt(n + lambda);
    
    // L * L^T = P (Cholesky)
    cmath_fx::FixedMatrix L;
    if (!P.cholesky(L)) {
        // 正定値でない場合は摂動
        cmath_fx::FixedMatrix P_reg = P + eye(n) * 1e-8;
        if (!P_reg.cholesky(L)) return false;
    }
    
    // Sigma points: x, x ± gamma*L
    sigma_pts.resize(n, 2*n+1);
    // ... 詳細は現在の実装参照
}
```

#### テスト

```bash
# 修正前後で数値一致確認
cd kalman/cpp/build
build_mex({'mex_ukf_sigma_points'})
clear mex
run_simulation(42, true)
# Results/estimation_matlab.csv vs Results/estimation_mex.csv で差異 < 1e-5
```

---

### Phase 5.4: EKF の Eigen 参照削除

**目標**: EKF ファイルから Eigen インクルードを廃止  
**期間**: <1日  
**ファイル**:
- [kalman/cpp/include/EKF/ekf.hpp](../include/EKF/ekf.hpp)
- [kalman/cpp/EKF/ekf.hpp](../EKF/ekf.hpp)

#### 修正内容

```diff
// ekf.hpp
- #include <Eigen/Dense>
- using Matrix = Eigen::MatrixXd;
- using Vector = Eigen::VectorXd;

+ #include "../Common/Math/fixed_matrix.hpp"
+ using Matrix = cmath_fx::FixedMatrix;
+ using Vector = cmath_fx::Vector<NDIM, 1>;
```

**注**: EKF は現在使用されていないため、簡易修正のみ

---

### Phase 5.5: ビルド検証と最適化

**目標**: 全 MEX のコンパイル確認と性能検証  
**期間**: 1-2日  
**手順**:

```matlab
% kalman/cpp/build/build_mex.m
cd kalman/cpp/build
build_mex()                           % 全体ビルド（Eigen なし）
clear mex

% テスト実行
cd ../../..
run_batch_10sets(true)                % MEX 優先実行
run_batch_10sets(false)               % MATLAB 参照実行

% 差分検証
diff_result = compare_batch_results(); % Results/batch_10sets_log.txt 比較
assert(max(diff_result) < TOLERANCE, 'Numerical divergence detected');
```

---

## 🛠️ 実装チェックリスト

### Phase 5.1: Cholesky 分解

- [ ] `FixedMatrix::cholesky()` 実装
- [ ] `FixedMatrix::cholesky_sqrt()` 実装
- [ ] ユニットテスト `test_cholesky.m` 作成・合格
- [ ] 数値安定性テスト（病的行列）実施

### Phase 5.2: 固有値分解

- [ ] QR 法アルゴリズム実装
- [ ] `FixedMatrix::eigen_decompose()` 実装
- [ ] ユニットテスト `test_eigen.m` 作成・合格
- [ ] 収束性テスト実施

### Phase 5.3: UKF Sigma Points 移行

- [ ] Eigen `#include` 削除
- [ ] `cholesky_sqrt()` 置き換え
- [ ] ビルド成功確認
- [ ] シミュレーション数値検証（MATLAB vs MEX < 1e-5）

### Phase 5.4: EKF 参照削除

- [ ] Eigen `#include` 削除（両ファイル）
- [ ] `cmath_fx` に置き換え
- [ ] ビルド成功確認

### Phase 5.5: 全体検証

- [ ] 全 MEX ビルド成功
- [ ] バッチ 10 セット実行（MEX/MATLAB 両方）
- [ ] 数値差異 < 許容値確認
- [ ] ドキュメント更新

---

## 📦 依存関係マップ

```
Eigen（廃止予定）
  ├── ukf_sigma_points.hpp
  │   └── [Phase 5.3] cholesky_sqrt() に置き換え
  └── ekf.hpp
      └── [Phase 5.4] 簡易置き換え

cmath_fx（拡張）
  ├── fixed_matrix.hpp
  │   ├── [Phase 5.1] cholesky(), cholesky_sqrt() 追加
  │   └── [Phase 5.2] eigen_decompose() 追加
  ├── quaternion.hpp        ✅ 既存
  ├── rotation_compute.hpp  ✅ 既存
  └── math_utils.hpp        ✅ 既存

MEX インターフェース
  ├── mex_ukf_sigma_points
  ├── mex_meukf_step_v2     ✅ 既に cmath_fx 使用中
  ├── mex_eskf_step         ✅ 既に cmath_fx 使用中
  └── mex_common_lib        → cholesky, eigen_decompose ハンドラ追加
```

---

## 🎬 実行順序

```
Week 1
├─ Mon: Phase 5.1 (Cholesky 実装)
├─ Tue: Phase 5.2 (固有値分解実装)
├─ Wed: Phase 5.3 (UKF 移行)
├─ Thu: Phase 5.4 (EKF 参照削除)
└─ Fri: Phase 5.5 (全体検証・ドキュメント)

各フェーズ後: build → clear mex → test
```

---

## 📋 テスト戦略

### ユニットテスト

| テスト | ファイル | 目的 |
|---|---|---|
| `test_cholesky.m` | 正定値行列の分解 | Cholesky 正確性 |
| `test_eigen.m` | 対称行列の固有値 | 固有値分解精度 |
| `test_ukf_sigma.m` | Sigma Points 生成 | UKF 数値一致 |

### 統合テスト

```matlab
run_batch_10sets(true)   % MEX（Eigen なし）
run_batch_10sets(false)  % MATLAB
% 差異チェック: max(|MATLAB - MEX|) < 1e-5 @全セット
```

---

## 📊 期待される効果

| 指標 | 現状 | 目標 |
|---|---|---|
| **外部依存数** | 1 (Eigen) | 0 |
| **ビルド時間** | ~30秒 | ~20秒（推定） |
| **ライブラリサイズ** | ~5MB (Eigen) | ~2MB（削減） |
| **デバッグ容易性** | 低 | 高（自作コード） |
| **MEX 数値精度** | 既に良好 | 変わらず（互換性） |

---

## 🚀 Phase 6 への前提

Phase 5 完了後に初めて Phase 6 実装可能：

```
Phase 6 計画（仮）
├─ 高度なセンサ融合（ビジョンベース）
├─ マルチモード切り替え（MEUKF/ESKF）
├─ リアルタイム性能最適化
└─ 組込み移植対応（ARM/DSP）
```

---

## 参考資料

- [fixed_matrix.hpp](../Common/Math/fixed_matrix.hpp) — 自作ライブラリコア
- [ukf_sigma_points.hpp](../UKF/Core/ukf_sigma_points.hpp) — 現 Eigen 依存コード
- [MEX_STATUS_REPORT.md](./MEX_STATUS_REPORT.md) — MEX ビルド状況

---

**作成者**: GitHub Copilot  
**最終確認**: 2025年12月21日
