# PHASE 5 実装ガイド

**対象**: Eigen 廃止と自作ライブラリ完成  
**期間**: 1 週間（5 営業日）  
**難易度**: ⭐⭐⭐（中程度）

---

## 📍 段階的実装指南

### ステップ 1: Cholesky 分解の実装（2 時間）

#### 1.1 理論背景

Cholesky 分解：対称正定値行列 $A$ を下三角行列 $L$ で分解
$$A = L L^T$$

**性質**:
- 計算量: $O(n^3/3)$（LU 分解より高速）
- 数値安定性: 優秀
- 応用: 共分散行列の平方根計算（UKF シグマポイント）

#### 1.2 実装コード

[fixed_matrix.hpp](../Common/Math/fixed_matrix.hpp) の `FixedMatrix` struct に追加：

```cpp
// In struct FixedMatrix { ... }

/**
 * Cholesky decomposition: A = L * L^T
 * Input: A (symmetric positive definite)
 * Output: L (lower triangular, row-major storage)
 * Return: true if successful, false if not positive definite
 */
bool cholesky(FixedMatrix& L) const {
    if (rows != cols) return false;
    int n = rows;
    L.resize(n, n);
    
    // Initialize L
    for(int i = 0; i < n*n; ++i) L.data[i] = 0.0f;
    
    // Doolittle algorithm
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j <= i; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < j; ++k) {
                sum += L(i, k) * L(j, k);
            }
            
            if (i == j) {
                float diag = (*this)(i, i) - sum;
                if (diag <= 1e-12f) {
                    // Not positive definite
                    return false;
                }
                L(i, i) = std::sqrt(diag);
            } else {
                L(i, j) = ((*this)(i, j) - sum) / L(j, j);
            }
        }
    }
    
    return true;
}

/**
 * Cholesky decomposition and square root: sqrt(A) = L
 * Convenience wrapper for sigma point generation
 */
bool cholesky_sqrt(FixedMatrix& sqrt_A) const {
    return this->cholesky(sqrt_A);
}
```

#### 1.3 テストコード

[kalman/cpp/MEUKF/test_cholesky.m](../MEUKF/test_cholesky.m) (新規作成)

```matlab
function test_cholesky()
    % Unit test for Cholesky decomposition
    
    % Test 1: Simple 2x2 positive definite matrix
    A = [4, 2; 2, 3];
    L_expected = [2, 0; 1, sqrt(2)];
    
    % Call MEX (assuming mex_common_lib has 'cholesky' handler)
    L = mex_common_lib('cholesky', A);
    
    % Verify: A = L * L^T
    A_reconstructed = L * L';
    err1 = norm(A - A_reconstructed, 'fro');
    fprintf('Test 1 (2x2): Error = %.2e\n', err1);
    assert(err1 < 1e-6, 'Test 1 failed');
    
    % Test 2: 3x3 matrix (from covariance example)
    P3 = [0.1, 0.01, 0.002; 
          0.01, 0.09, 0.003; 
          0.002, 0.003, 0.08];
    L3 = mex_common_lib('cholesky', P3);
    err2 = norm(P3 - L3*L3', 'fro');
    fprintf('Test 2 (3x3): Error = %.2e\n', err2);
    assert(err2 < 1e-6, 'Test 2 failed');
    
    % Test 3: Sigma point scale (15x15 covariance)
    P15 = rand(15, 15);
    P15 = P15' * P15;  % Make symmetric positive definite
    P15_scaled = P15 * 0.1;  % Sigma point scale
    
    L15 = mex_common_lib('cholesky', P15_scaled);
    err3 = norm(P15_scaled - L15*L15', 'fro');
    fprintf('Test 3 (15x15): Error = %.2e\n', err3);
    assert(err3 < 1e-5, 'Test 3 failed');
    
    % Test 4: Near-singular matrix (should fail gracefully)
    P_bad = [1, 1; 1, 1+1e-13];  % Numerically singular
    try
        L_bad = mex_common_lib('cholesky', P_bad);
        fprintf('Test 4: Singular matrix handled (expected behavior)\n');
    catch
        fprintf('Test 4: Exception thrown (acceptable)\n');
    end
    
    fprintf('All Cholesky tests passed!\n');
end
```

#### 1.4 MEX ハンドラ登録

[kalman/cpp/MEX/mex_common_lib.cpp](../MEX/mex_common_lib.cpp) に追加：

```cpp
void handle_cholesky(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1 || nlhs < 1) {
        mexErrMsgIdAndTxt("mex_common_lib:nrhs", "Usage: L = mex_common_lib('cholesky', A)");
    }
    
    cm A = matToFixed(prhs[0]);
    cm L;
    
    if (!A.cholesky(L)) {
        mexErrMsgIdAndTxt("mex_common_lib:cholesky", 
            "Matrix is not positive definite");
    }
    
    plhs[0] = fixedToMat(L);
}

// In mexFunction dispatch:
if (strcmp(cmd, "cholesky") == 0) {
    handle_cholesky(nlhs, plhs, nrhs-1, prhs+1);
    return;
}
```

---

### ステップ 2: 固有値分解の実装（4 時間）

#### 2.1 理論背景

**QR 法による固有値分解**

対称行列 $A$ の固有値・固有ベクトルを計算：
$$A = V \Lambda V^T$$

ここで $V$ は固有ベクトル、$\Lambda$ は固有値の対角行列。

**QR 反復プロセス**:
```
for k = 1 to max_iter:
    A_k = Q_k * R_k      (QR分解)
    A_{k+1} = R_k * Q_k  (再構成)
    if ||A_{k+1} - A_k|| < tol: break
```

#### 2.2 QR 分解の実装

```cpp
// Helper: QR decomposition (Gram-Schmidt)
// Input: A (m x n, m >= n)
// Output: Q (m x n orthogonal), R (n x n upper triangular)
bool qr_decompose(const FixedMatrix& A, FixedMatrix& Q, FixedMatrix& R) {
    int m = A.rows;
    int n = A.cols;
    
    if (m < n) return false;
    
    Q.resize(m, n);
    R.resize(n, n);
    
    // Gram-Schmidt orthogonalization
    for (int j = 0; j < n; ++j) {
        // Extract column j
        FixedMatrix a_j(m, 1);
        for (int i = 0; i < m; ++i) {
            a_j(i, 0) = A(i, j);
        }
        
        // Orthogonalize against previous columns
        for (int i = 0; i < j; ++i) {
            FixedMatrix q_i(m, 1);
            for (int k = 0; k < m; ++k) {
                q_i(k, 0) = Q(k, i);
            }
            
            // R[i,j] = Q[:,i]^T * A[:,j]
            float dot = 0.0f;
            for (int k = 0; k < m; ++k) {
                dot += q_i(k, 0) * a_j(k, 0);
            }
            R(i, j) = dot;
            
            // a_j = a_j - R[i,j] * Q[:,i]
            for (int k = 0; k < m; ++k) {
                a_j(k, 0) -= dot * q_i(k, 0);
            }
        }
        
        // R[j,j] = ||a_j||
        float norm = 0.0f;
        for (int i = 0; i < m; ++i) {
            norm += a_j(i, 0) * a_j(i, 0);
        }
        norm = std::sqrt(norm);
        
        if (norm < 1e-12f) {
            return false;  // Rank deficient
        }
        
        R(j, j) = norm;
        
        // Q[:,j] = a_j / norm
        for (int i = 0; i < m; ++i) {
            Q(i, j) = a_j(i, 0) / norm;
        }
    }
    
    return true;
}
```

#### 2.3 固有値分解メイン関数

```cpp
/**
 * Eigenvalue decomposition for symmetric matrix using QR iteration
 * Input: A (symmetric n x n)
 * Output: eigenvalues (n x 1), eigenvectors (n x n where cols are eigenvectors)
 * Return: true if converged, false otherwise
 */
bool eigen_decompose(
    FixedMatrix& eigenvalues,
    FixedMatrix& eigenvectors,
    int max_iter = 100,
    float tol = 1e-8f
) const {
    if (rows != cols) return false;
    int n = rows;
    
    // Initialize: A0 = A, V0 = I
    FixedMatrix A_k = *this;
    FixedMatrix V_k;
    V_k.resize(n, n);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            V_k(i, j) = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    // QR iteration
    for (int iter = 0; iter < max_iter; ++iter) {
        FixedMatrix Q, R;
        
        if (!qr_decompose(A_k, Q, R)) {
            return false;
        }
        
        // A_{k+1} = R * Q
        FixedMatrix A_next(n, n);
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                float sum = 0.0f;
                for (int k = 0; k < n; ++k) {
                    sum += R(i, k) * Q(k, j);
                }
                A_next(i, j) = sum;
            }
        }
        
        // V_{k+1} = V_k * Q
        FixedMatrix V_next(n, n);
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                float sum = 0.0f;
                for (int k = 0; k < n; ++k) {
                    sum += V_k(i, k) * Q(k, j);
                }
                V_next(i, j) = sum;
            }
        }
        
        // Check convergence: sum of squared off-diagonal elements
        float off_diag_norm = 0.0f;
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                if (i != j) {
                    off_diag_norm += A_next(i, j) * A_next(i, j);
                }
            }
        }
        
        if (std::sqrt(off_diag_norm) < tol) {
            // Converged: extract eigenvalues and eigenvectors
            eigenvalues.resize(n, 1);
            eigenvectors = V_next;
            for (int i = 0; i < n; ++i) {
                eigenvalues(i, 0) = A_next(i, i);
            }
            return true;
        }
        
        A_k = A_next;
        V_k = V_next;
    }
    
    return false;  // Did not converge
}
```

#### 2.4 テストコード

[kalman/cpp/MEUKF/test_eigen.m](../MEUKF/test_eigen.m) (新規作成)

```matlab
function test_eigen()
    % Unit test for eigenvalue decomposition
    
    % Test 1: Simple 2x2 symmetric matrix
    A = [3, 1; 1, 2];
    [eigenvalues, eigenvectors] = mex_common_lib('eigen_decompose', A);
    
    % Verify: A * v = lambda * v
    for i = 1:2
        v = eigenvectors(:, i);
        lambda = eigenvalues(i, 1);
        residual = norm(A*v - lambda*v);
        fprintf('Test 1, eigenvector %d: residual = %.2e\n', i, residual);
        assert(residual < 1e-5, sprintf('Test 1.%d failed', i));
    end
    
    % Test 2: 3x3 matrix
    P = [0.1, 0.01, 0.002; 
         0.01, 0.09, 0.003; 
         0.002, 0.003, 0.08];
    [eig_vals, eig_vecs] = mex_common_lib('eigen_decompose', P);
    
    % Check: P = V * Lambda * V^T
    Lambda = diag(eig_vals);
    P_reconstructed = eig_vecs * Lambda * eig_vecs';
    err = norm(P - P_reconstructed, 'fro');
    fprintf('Test 2 (3x3 reconstruction): Error = %.2e\n', err);
    assert(err < 1e-5, 'Test 2 failed');
    
    % Test 3: Large 15x15 covariance
    P15 = rand(15, 15);
    P15 = (P15 + P15') / 2;  % Symmetrize
    P15 = P15 + eye(15) * max(eig(P15)) * 0.1;  % Ensure positive definite
    
    [eig_vals15, eig_vecs15] = mex_common_lib('eigen_decompose', P15);
    
    % Check eigenvalues are positive
    assert(all(eig_vals15 > 1e-10), 'Test 3: Negative eigenvalue');
    
    % Check reconstruction
    Lambda15 = diag(eig_vals15);
    P15_reconstructed = eig_vecs15 * Lambda15 * eig_vecs15';
    err15 = norm(P15 - P15_reconstructed, 'fro');
    fprintf('Test 3 (15x15 reconstruction): Error = %.2e\n', err15);
    assert(err15 < 1e-4, 'Test 3 failed');
    
    fprintf('All eigenvalue decomposition tests passed!\n');
end
```

---

### ステップ 3: UKF Sigma Points の移行（2 時間）

#### 3.1 修正個所の特定

[kalman/cpp/UKF/Core/ukf_sigma_points.hpp](../UKF/Core/ukf_sigma_points.hpp):

```cpp
// ❌ Before:
#include <Eigen/Dense>
using Matrix = Eigen::MatrixXf;
using Vector = Eigen::VectorXf;

// ✅ After:
#include "../../Common/Math/fixed_matrix.hpp"
using Matrix = cmath_fx::FixedMatrix;
using Vector = cmath_fx::FixedMatrix;  // Column vector
```

#### 3.2 Cholesky 呼び出しの置き換え

[kalman/cpp/UKF/Core/ukf_sigma_points.cpp](../UKF/Core/ukf_sigma_points.cpp)

```cpp
// ❌ Before:
Eigen::LLT<Matrix> llt(P_scaled);
if (llt.info() != Eigen::Success) {
    // Fall back with regularization
}
sqrtP = llt.matrixL();

// ✅ After:
if (!P_scaled.cholesky_sqrt(sqrtP)) {
    // Add regularization
    Matrix P_reg = P_scaled;
    for (int i = 0; i < P_reg.rows; ++i) {
        P_reg(i, i) += 1e-8f;
    }
    if (!P_reg.cholesky_sqrt(sqrtP)) {
        // Eigenvalue-based regularization
        Matrix eig_vals, eig_vecs;
        if (!P_scaled.eigen_decompose(eig_vals, eig_vecs)) {
            return false;
        }
        // ... handle eigenvalue-based sqrt
    }
}
```

#### 3.3 ビルドと検証

```bash
cd kalman/cpp/build
build_mex({'mex_ukf_sigma_points'})
clear mex
addpath(fullfile(pwd, '..', 'bin'))

% MATLAB から
cd ../..
run_simulation(42, true)
```

---

### ステップ 4: ドキュメント更新（1 時間）

#### 4.1 更新対象

- [PHASE5_EIGEN_MIGRATION_PLAN.md](./PHASE5_EIGEN_MIGRATION_PLAN.md) — 本計画書
- [.github/copilot-instructions.md](../../.github/copilot-instructions.md) — AI ガイド
- [kalman/cpp/markdown/README.md](./README.md) — ライブラリドキュメント

#### 4.2 追記内容

```markdown
## Phase 5: Eigen 廃止完了（2025/12/??）

✅ **完了した項目**:
1. Cholesky 分解実装（`cmath_fx::FixedMatrix::cholesky()`）
2. 固有値分解実装（QR 法）
3. UKF Sigma Points の移行完了
4. EKF の Eigen 参照削除
5. 全 MEX ビルド成功

**利益**:
- 外部依存 0（Eigen 廃止）
- ライブラリサイズ削減
- デバッグ容易性向上
- 組込み環境への移植容易化
```

---

## 🧪 統合テスト手順

### テスト実行シーケンス

```bash
% Step 1: ビルド
cd kalman/cpp/build
build_mex()
clear mex

% Step 2: ユニットテスト
cd ../MEUKF
matlab -nodisplay -r "test_cholesky; test_eigen; exit"

% Step 3: 統合テスト（UKF Sigma Points）
cd ../../..
run_simulation(42, true)

% Step 4: 差分検証
diff_threshold = 1e-5;
load Results/estimation_matlab.csv
load Results/estimation_mex.csv
diff = abs(estimation_matlab - estimation_mex);
assert(max(diff(:)) < diff_threshold, 'Divergence detected');
```

### 期待される出力

```
✅ test_cholesky passed
✅ test_eigen passed
✅ run_simulation completed
✅ Numerical difference: 2.3e-6 (< 1e-5 threshold)
✅ All tests passed!
```

---

## ⚠️ よくある落とし穴と対応

| 問題 | 原因 | 対応 |
|---|---|---|
| 「正定値ではない」エラー | 数値誤差の蓄積 | regularization: `A + eps*I` 追加 |
| QR 反復が収束しない | 悪条件行列 | max_iter 増加、tol 緩和 |
| ビルドが失敗 | ヘッダー参照エラー | `#include` パス確認 |
| MEX キャッシュ問題 | 古い MEX をロード | `clear mex` 必須 |

---

## 📝 チェックリスト（実行用）

```
[ ] Phase 5.1: Cholesky 実装
    [ ] fixed_matrix.hpp に cholesky() 追加
    [ ] mex_common_lib.cpp にハンドラ登録
    [ ] test_cholesky.m で合格

[ ] Phase 5.2: 固有値分解実装
    [ ] qr_decompose() 実装
    [ ] eigen_decompose() 実装
    [ ] test_eigen.m で合格

[ ] Phase 5.3: UKF 移行
    [ ] ukf_sigma_points.hpp の Eigen 参照削除
    [ ] ukf_sigma_points.cpp の Cholesky 置き換え
    [ ] build_mex({mex_ukf_sigma_points}) 成功
    [ ] run_simulation(42, true) で数値一致確認

[ ] Phase 5.4: EKF 参照削除
    [ ] ekf.hpp (両ファイル) の Eigen 参照削除
    [ ] build_mex() 全体成功

[ ] Phase 5.5: 最終検証
    [ ] batch_10sets(true) 実行
    [ ] batch_10sets(false) 実行
    [ ] 差分検証: max_diff < 1e-5
    [ ] ドキュメント更新
```

---

**作成者**: GitHub Copilot  
**更新日**: 2025年12月21日
