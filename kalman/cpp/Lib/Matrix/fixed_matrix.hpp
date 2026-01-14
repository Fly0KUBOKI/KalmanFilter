#pragma once
#ifndef LIB_MATRIX_FIXED_MATRIX_HPP
#define LIB_MATRIX_FIXED_MATRIX_HPP

// Implementation: このヘッダー内に実装含む（テンプレート実装）
// Moved from Inc/Common/Math/fixed_matrix.hpp to Lib/Matrix/fixed_matrix.hpp

#include <cmath>
#include "../Common/inc/Math/portable_math.hpp"
#include <cstring>
#include <cassert>
#include <algorithm>

namespace cmath_fx {

// Fixed-size matrix class
template <int R, int C, typename T = float>
struct Matrix {
    static const int Rows = R;
    static const int Cols = C;
    T data[R * C];

    Matrix() {
        std::memset(data, 0, sizeof(data));
    }

    static Matrix Zero() {
        Matrix m;
        return m;
    }

    static Matrix Identity() {
        static_assert(R == C, "Identity matrix must be square");
        Matrix m;
        for (int i = 0; i < R; ++i) m(i, i) = static_cast<T>(1);
        return m;
    }

    inline T& operator()(int r, int c) {
        assert(r >= 0 && r < R && c >= 0 && c < C);
        return data[r * C + c];
    }

    inline const T& operator()(int r, int c) const {
        assert(r >= 0 && r < R && c >= 0 && c < C);
        return data[r * C + c];
    }

    // 加算
    Matrix<R, C, T> operator+(const Matrix<R, C, T>& other) const {
        Matrix<R, C, T> res;
        for (int i = 0; i < R * C; ++i) res.data[i] = data[i] + other.data[i];
        return res;
    }

    // 減算
    Matrix<R, C, T> operator-(const Matrix<R, C, T>& other) const {
        Matrix<R, C, T> res;
        for (int i = 0; i < R * C; ++i) res.data[i] = data[i] - other.data[i];
        return res;
    }

    // スカラー倍
    Matrix<R, C, T> operator*(T scalar) const {
        Matrix<R, C, T> res;
        for (int i = 0; i < R * C; ++i) res.data[i] = data[i] * scalar;
        return res;
    }

    // 行列積
    template <int K>
    Matrix<R, K, T> operator*(const Matrix<C, K, T>& other) const {
        Matrix<R, K, T> res;
        for (int i = 0; i < R; ++i) {
            for (int j = 0; j < K; ++j) {
                T sum = 0;
                for (int k = 0; k < C; ++k) {
                    sum += (*this)(i, k) * other(k, j);
                }
                res(i, j) = sum;
            }
        }
        return res;
    }

    // 転置
    Matrix<C, R, T> transpose() const {
        Matrix<C, R, T> res;
        for (int i = 0; i < R; ++i) {
            for (int j = 0; j < C; ++j) {
                res(j, i) = (*this)(i, j);
            }
        }
        return res;
    }

    // 逆行列 (Gauss-Jordan) - 正方行列のみ
    bool inverse(Matrix<R, C, T>& inv) const {
        static_assert(R == C, "Inverse requires square matrix");
        Matrix<R, R * 2, T> aug;

        // 拡大係数行列 [A | I]
        for (int i = 0; i < R; ++i) {
            for (int j = 0; j < R; ++j) aug(i, j) = (*this)(i, j);
            aug(i, R + i) = static_cast<T>(1);
        }

        for (int i = 0; i < R; ++i) {
            // ピボット選択
            int pivot = i;
            T max_val = std::abs(aug(i, i));
            for (int k = i + 1; k < R; ++k) {
                if (std::abs(aug(k, i)) > max_val) {
                    max_val = std::abs(aug(k, i));
                    pivot = k;
                }
            }

            if (max_val < static_cast<T>(1e-12)) return false; // 特異行列

            // 行入れ替え
            if (pivot != i) {
                for (int j = 0; j < 2 * R; ++j) std::swap(aug(i, j), aug(pivot, j));
            }

            // 正規化
            T div = aug(i, i);
            for (int j = i; j < 2 * R; ++j) aug(i, j) /= div;

            // 消去
            for (int k = 0; k < R; ++k) {
                if (k != i) {
                    T factor = aug(k, i);
                    for (int j = i; j < 2 * R; ++j) aug(k, j) -= factor * aug(i, j);
                }
            }
        }

        // 結果の取り出し
        for (int i = 0; i < R; ++i) {
            for (int j = 0; j < R; ++j) inv(i, j) = aug(i, R + j);
        }
        return true;
    }

    // Cholesky decomposition (lower triangular matrix L where A = L*L^T)
    // Returns true if successful, false if matrix is not positive definite
    bool cholesky(Matrix<R, C, T>& L) const {
        static_assert(R == C, "Cholesky decomposition requires square matrix");
        L = Zero();

        for (int i = 0; i < R; ++i) {
            for (int j = 0; j <= i; ++j) {
                T sum = static_cast<T>(0);
                for (int k = 0; k < j; ++k) {
                    sum += L(i, k) * L(j, k);
                }

                if (i == j) {
                    T val = (*this)(i, i) - sum;
                    if (val <= static_cast<T>(1e-12)) return false; // Not positive definite
                    L(i, i) = common::math::portable_sqrt(val);
                } else {
                    if (std::abs(L(j, j)) < static_cast<T>(1e-12)) return false;
                    L(i, j) = ((*this)(i, j) - sum) / L(j, j);
                }
            }
        }
        return true;
    }
};

// ベクトル型定義
template <int N, typename T = float>
using Vector = Matrix<N, 1, T>;

// Runtime-sized matrix with fixed maximum capacity (for MEX interfacing)
const int MAX_N = 20;

struct FixedMatrix {
    int rows;
    int cols;
    float data[MAX_N * MAX_N];

    FixedMatrix() : rows(0), cols(0) {
        std::memset(data, 0, sizeof(data));
    }

    FixedMatrix(int r, int c) : rows(r), cols(c) {
        assert(r <= MAX_N && c <= MAX_N);
        std::memset(data, 0, sizeof(data));
    }

    void resize(int r, int c) {
        assert(r <= MAX_N && c <= MAX_N);
        rows = r;
        cols = c;
        std::memset(data, 0, sizeof(data));
    }

    inline float& operator()(int r, int c) {
        return data[r * cols + c];
    }

    inline const float& operator()(int r, int c) const {
        return data[r * cols + c];
    }

    // Conversion to Template Matrix
    template <int R, int C>
    Matrix<R, C, float> toMatrix() const {
        assert(rows == R && cols == C);
        Matrix<R, C, float> m;
        for (int i = 0; i < R; ++i) {
            for (int j = 0; j < C; ++j) {
                m(i, j) = (*this)(i, j);
            }
        }
        return m;
    }

    // Assignment from Template Matrix
    template <int R, int C>
    void fromMatrix(const Matrix<R, C, float>& m) {
        rows = R;
        cols = C;
        for (int i = 0; i < R; ++i) {
            for (int j = 0; j < C; ++j) {
                (*this)(i, j) = m(i, j);
            }
        }
    }

    // Basic operations needed for MEX
    FixedMatrix transpose() const {
        FixedMatrix res(cols, rows);
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                res(j, i) = (*this)(i, j);
            }
        }
        return res;
    }

    FixedMatrix operator+(const FixedMatrix& other) const {
        assert(rows == other.rows && cols == other.cols);
        FixedMatrix res(rows, cols);
        for (int i = 0; i < rows * cols; ++i) res.data[i] = data[i] + other.data[i];
        return res;
    }

    FixedMatrix operator-(const FixedMatrix& other) const {
        assert(rows == other.rows && cols == other.cols);
        FixedMatrix res(rows, cols);
        for (int i = 0; i < rows * cols; ++i) res.data[i] = data[i] - other.data[i];
        return res;
    }

    FixedMatrix operator*(const FixedMatrix& other) const {
        assert(cols == other.rows);
        FixedMatrix res(rows, other.cols);
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < other.cols; ++j) {
                float sum = 0;
                for (int k = 0; k < cols; ++k) {
                    sum += (*this)(i, k) * other(k, j);
                }
                res(i, j) = sum;
            }
        }
        return res;
    }

    bool inverse(FixedMatrix& inv) const {
        assert(rows == cols);
        int n = rows;
        inv.resize(n, n);

        // Simple Gauss-Jordan (copy-paste logic adapted for runtime size)
        float aug[MAX_N][MAX_N * 2];
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) aug[i][j] = (*this)(i, j);
            for (int j = n; j < 2 * n; ++j) aug[i][j] = (j - n == i) ? 1.0f : 0.0f;
        }

        for (int i = 0; i < n; ++i) {
            int pivot = i;
            float max_val = std::abs(aug[i][i]);
            for (int k = i + 1; k < n; ++k) {
                if (std::abs(aug[k][i]) > max_val) {
                    max_val = std::abs(aug[k][i]);
                    pivot = k;
                }
            }
            if (max_val < 1e-12f) return false;

            if (pivot != i) {
                for (int j = 0; j < 2 * n; ++j) std::swap(aug[i][j], aug[pivot][j]);
            }

            float div = aug[i][i];
            for (int j = i; j < 2 * n; ++j) aug[i][j] /= div;

            for (int k = 0; k < n; ++k) {
                if (k != i) {
                    float factor = aug[k][i];
                    for (int j = i; j < 2 * n; ++j) aug[k][j] -= factor * aug[i][j];
                }
            }
        }

        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                inv(i, j) = aug[i][n + j];
            }
        }
        return true;
    }

    // Cholesky decomposition (lower triangular matrix L where A = L*L^T)
    // Returns true if successful, false if matrix is not positive definite
    bool cholesky(FixedMatrix& L) const {
        if (rows != cols) return false;
        int n = rows;
        L.resize(n, n);

        // Initialize L to zero
        std::memset(L.data, 0, sizeof(L.data));

        for (int i = 0; i < n; ++i) {
            for (int j = 0; j <= i; ++j) {
                float sum = 0.0f;
                for (int k = 0; k < j; ++k) {
                    sum += L(i, k) * L(j, k);
                }

                    if (i == j) {
                    float val = (*this)(i, i) - sum;
                    if (val <= 1e-12f) return false; // Not positive definite
                    L(i, i) = common::math::portable_sqrt(val);
                } else {
                    if (std::abs(L(j, j)) < 1e-12f) return false;
                    L(i, j) = ((*this)(i, j) - sum) / L(j, j);
                }
            }
        }
        return true;
    }
};

// ========================================
// Decomposition Namespace (Cholesky, etc.)
// ========================================
namespace decomp {

// Generic Cholesky Decomposition (NxN)
template <int N, typename T = float>
bool cholesky(const Matrix<N, N, T>& A, Matrix<N, N, T>& L) {
    L = Matrix<N, N, T>::Zero();

    for (int i = 0; i < N; ++i) {
        for (int j = 0; j <= i; ++j) {
            T sum = static_cast<T>(0);
            for (int k = 0; k < j; ++k) {
                sum += L(i, k) * L(j, k);
            }

            if (i == j) {
                T val = A(i, i) - sum;
                if (val <= static_cast<T>(1e-12)) return false;
                L(i, i) = common::math::portable_sqrt(val);
            } else {
                if (std::abs(L(j, j)) < static_cast<T>(1e-12)) return false;
                L(i, j) = (A(i, j) - sum) / L(j, j);
            }
        }
    }
    return true;
}

// Robust Cholesky with multi-stage fallback
template <int N, typename T = float>
bool cholesky_robust(Matrix<N, N, T>& A, Matrix<N, N, T>& L) {
    // 1. Symmetrize
    for (int i = 0; i < N; ++i) {
        for (int j = i + 1; j < N; ++j) {
            T avg = (A(i, j) + A(j, i)) * static_cast<T>(0.5);
            A(i, j) = avg;
            A(j, i) = avg;
        }
    }

    // 2. Check minimum diagonal as proxy for eigenvalue
    T min_diag = A(0, 0);
    for (int i = 1; i < N; ++i) {
        if (A(i, i) < min_diag) min_diag = A(i, i);
    }

    // 3. Regularize if not positive definite
    if (min_diag <= static_cast<T>(0)) {
        T reg = std::abs(min_diag) + static_cast<T>(1e-6);
        for (int i = 0; i < N; ++i) A(i, i) += reg;
    }

    // 4. Try standard Cholesky
    if (cholesky<N, T>(A, L)) return true;

    // 5. Stronger regularization
    for (int i = 0; i < N; ++i) A(i, i) += static_cast<T>(1e-4);
    if (cholesky<N, T>(A, L)) return true;

    // 6. Fallback: diagonal sqrt approximation
    L = Matrix<N, N, T>::Zero();
    for (int i = 0; i < N; ++i) {
        L(i, i) = common::math::portable_sqrt(std::max(static_cast<T>(0), A(i, i)));
    }
    return true; // succeed with approximation
}

// 3x3 optimized Cholesky
template <typename T>
bool cholesky_3x3_optimized(const Matrix<3, 3, T>& A, Matrix<3, 3, T>& L) {
    L = Matrix<3, 3, T>::Zero();

    if (A(0, 0) <= static_cast<T>(1e-12)) return false;
    L(0, 0) = common::math::portable_sqrt(A(0, 0));

    L(1, 0) = A(1, 0) / L(0, 0);
    T val11 = A(1, 1) - L(1, 0) * L(1, 0);
    if (val11 <= static_cast<T>(1e-12)) return false;
    L(1, 1) = common::math::portable_sqrt(val11);

    L(2, 0) = A(2, 0) / L(0, 0);
    L(2, 1) = (A(2, 1) - L(2, 0) * L(1, 0)) / L(1, 1);
    T val22 = A(2, 2) - L(2, 0) * L(2, 0) - L(2, 1) * L(2, 1);
    if (val22 <= static_cast<T>(1e-12)) return false;
    L(2, 2) = common::math::portable_sqrt(val22);

    return true;
}

// Overload for 3x3 that dispatches to the optimized implementation.
// This provides the effect of a template specialization without using
// C++17 `if constexpr` or function template partial specialization.
template <typename T>
bool cholesky(const Matrix<3, 3, T>& A, Matrix<3, 3, T>& L) {
    return cholesky_3x3_optimized<T>(A, L);
}

} // namespace decomp

// ========================================
// Inverse Namespace (Matrix inversion)
// ========================================
namespace inv {

// Analytic inverse for 3x3 matrices
template <typename T>
bool inverse_3x3_analytic(const Matrix<3, 3, T>& A, Matrix<3, 3, T>& A_inv) {
    T det = A(0,0) * (A(1,1)*A(2,2) - A(2,1)*A(1,2))
          - A(0,1) * (A(1,0)*A(2,2) - A(2,0)*A(1,2))
          + A(0,2) * (A(1,0)*A(2,1) - A(2,0)*A(1,1));

    if (std::abs(det) < static_cast<T>(1e-10)) return false;
    T inv_det = static_cast<T>(1.0) / det;

    A_inv(0,0) = (A(1,1)*A(2,2) - A(2,1)*A(1,2)) * inv_det;
    A_inv(0,1) = (A(0,2)*A(2,1) - A(0,1)*A(2,2)) * inv_det;
    A_inv(0,2) = (A(0,1)*A(1,2) - A(0,2)*A(1,1)) * inv_det;
    A_inv(1,0) = (A(1,2)*A(2,0) - A(1,0)*A(2,2)) * inv_det;
    A_inv(1,1) = (A(0,0)*A(2,2) - A(0,2)*A(2,0)) * inv_det;
    A_inv(1,2) = (A(1,0)*A(0,2) - A(0,0)*A(1,2)) * inv_det;
    A_inv(2,0) = (A(1,0)*A(2,1) - A(2,0)*A(1,1)) * inv_det;
    A_inv(2,1) = (A(2,0)*A(0,1) - A(0,0)*A(2,1)) * inv_det;
    A_inv(2,2) = (A(0,0)*A(1,1) - A(1,0)*A(0,1)) * inv_det;

    return true;
}

// Generic inverse: use Matrix::inverse for arbitrary size
template <int N, typename T>
bool inverse(const Matrix<N, N, T>& A, Matrix<N, N, T>& inv) {
    return A.inverse(inv);
}

// Overload for 3x3 to use analytic inverse for better stability/performance
template <typename T>
bool inverse(const Matrix<3, 3, T>& A, Matrix<3, 3, T>& inv) {
    return inverse_3x3_analytic<T>(A, inv);
}

} // namespace inv

// ========================================
// Utils Namespace (Symmetrization, etc.)
// ========================================
namespace utils {

// Symmetrize matrix in-place (template)
template <int N, typename T>
void symmetrize(Matrix<N, N, T>& M) {
    for (int i = 0; i < N; ++i) {
        for (int j = i + 1; j < N; ++j) {
            T avg = (M(i, j) + M(j, i)) * static_cast<T>(0.5);
            M(i, j) = avg;
            M(j, i) = avg;
        }
    }
}

// Non-destructive copy version
template <int N, typename T>
Matrix<N, N, T> symmetrize_copy(const Matrix<N, N, T>& M) {
    Matrix<N, N, T> res = M;
    symmetrize<N, T>(res);
    return res;
}

// Symmetrize runtime FixedMatrix in-place
inline void symmetrize(FixedMatrix& M) {
    int n = M.rows;
    int m = M.cols;
    int N = (n < m) ? n : m;
    for (int i = 0; i < N; ++i) {
        for (int j = i + 1; j < N; ++j) {
            float avg = 0.5f * (M(i, j) + M(j, i));
            M(i, j) = avg;
            M(j, i) = avg;
        }
    }
}

// Ensure positive definite by regularizing diagonal if needed
template <int N, typename T>
void ensure_positive_definite(Matrix<N, N, T>& M, T min_eigenvalue = static_cast<T>(1e-8)) {
    symmetrize<N, T>(M);

    T min_diag = M(0, 0);
    for (int i = 1; i < N; ++i) {
        if (M(i, i) < min_diag) min_diag = M(i, i);
    }

    if (min_diag < min_eigenvalue) {
        T reg = min_eigenvalue - min_diag;
        for (int i = 0; i < N; ++i) M(i, i) += reg;
        symmetrize<N, T>(M);
    }
}

} // namespace utils

} // namespace cmath_fx

// ========================================
// Common Math Utilities (from matrix_operations.hpp)
// ========================================
namespace common {
namespace math {

using cm = cmath_fx::FixedMatrix;

// Enforce matrix symmetry
inline cm enforce_symmetry(const cm& M) {
    cm result = M;
    for (int i = 0; i < M.rows; ++i) {
        for (int j = 0; j < M.cols; ++j) {
            result(i,j) = 0.5f * (M(i,j) + M(j,i));
        }
    }
    return result;
}

// Compute innovation and its covariance S (dynamic-size version)
inline void compute_innovation_and_S(const cm& z, const cm& h, const cm& H,
                                     const cm& P_pred, const cm& R,
                                     cm& y, cm& S, cm& R_out) {
    y = z - h;
    cm HP = H * P_pred;
    cm HPHt = HP * H.transpose();
    S = HPHt + R;
    S = enforce_symmetry(S);
    R_out = R;
}

// 3x3 inverse wrapper
template<typename T>
inline bool invert3x3(const cmath_fx::Matrix<3,3,T>& A, cmath_fx::Matrix<3,3,T>& A_inv) {
    return cmath_fx::inv::inverse<3, T>(A, A_inv);
}

// Safe Cholesky: attempt robust methods from Matrix layer
inline bool safe_cholesky(const cm& A, cm& L) {
    if (A.rows != A.cols) return false;
    int n = A.rows;
    L.resize(n, n);

    // Try FixedMatrix cholesky first
    if (A.cholesky(L)) return true;

    cm B = A;
    // Symmetrize and delegate to matrix_decomposition robust routine
    for (int i = 0; i < n; ++i) for (int j = i+1; j < n; ++j) {
        float avg = 0.5f * (B(i,j) + B(j,i)); B(i,j) = avg; B(j,i) = avg;
    }

    // Try robust cholesky from Matrix layer (fallbacks internally)
    // Use dynamic path: convert cm to templated Matrix<N,N> not available, so use A.cholesky again
    if (B.cholesky(L)) return true;

    // Try small regularization
    float eps = 1e-8f;
    for (int attempt=0; attempt<4; ++attempt) {
        for (int i=0;i<n;++i) B(i,i) += eps;
        if (B.cholesky(L)) return true;
        eps *= 10.0f;
    }

    // Final fallback: diagonal
    for (int i=0;i<n;++i) for (int j=0;j<n;++j) L(i,j) = 0.0f;
    for (int i=0;i<n;++i) {
        float v = A(i,i); if (v < 0.0f) v = 0.0f; L(i,i) = common::math::portable_sqrt(v);
    }
    return true;
}

// Mahalanobis distance (dynamic FixedMatrix version)
inline float mahalanobis_distance_squared(const cm& innovation, const cm& S) {
    int n = innovation.rows;
    if (n == 0) return 0.0f;

    cm L(n,n);
    if (!safe_cholesky(S, L)) {
        float max_var = 0.0f;
        for (int i=0;i<n;++i) if (S(i,i) > max_var) max_var = S(i,i);
        if (max_var < 1e-9f) max_var = 1.0f;
        float norm_sq = 0.0f;
        for (int i=0;i<n;++i) norm_sq += innovation(i,0) * innovation(i,0);
        return norm_sq / max_var;
    }

    // Solve L * y = innovation  (forward substitution)
    cm y_vec(n,1);
    for (int i=0;i<n;++i) {
        float sum = 0.0f;
        for (int j=0;j<i;++j) sum += L(i,j) * y_vec(j,0);
        float d = L(i,i);
        if (fabsf(d) < 1e-12f) d = 1e-12f;
        y_vec(i,0) = (innovation(i,0) - sum) / d;
    }

    float dist_sq = 0.0f;
    for (int i=0;i<n;++i) dist_sq += y_vec(i,0) * y_vec(i,0);
    return dist_sq;
}

// 3x3 skew-symmetric builder (vector v 3x1)
inline cm skew_symmetric(const cm& v) {
    cm S; S.resize(3,3);
    float vx = v(0,0); float vy = v(1,0); float vz = v(2,0);
    S(0,0)=0.0f; S(0,1)=-vz; S(0,2)=vy;
    S(1,0)=vz; S(1,1)=0.0f; S(1,2)=-vx;
    S(2,0)=-vy; S(2,1)=vx; S(2,2)=0.0f;
    return S;
}

} // namespace math
} // namespace common

#endif // LIB_MATRIX_FIXED_MATRIX_HPP
