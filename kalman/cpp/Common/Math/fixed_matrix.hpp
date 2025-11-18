#pragma once
// fixed_matrix.hpp
// 動的確保を行わない固定長行列/ベクトルライブラリ
// 設計: 最大次元をコンパイル時に定義し、内部は固定配列で保持する

#include <cmath>
#include <cstring>
#include <cassert>
#include <algorithm>

namespace cmath_fx {

static const int MAX_N = 15; // 最大状態次元（ESKF 用）
static const int MAX_M = 15; // 最大観測次元（必要に応じて調整）

struct FixedMatrix {
    int rows;
    int cols;
    float data[MAX_N * MAX_N]; // 使用しない領域は0にする

    FixedMatrix() : rows(0), cols(0) { std::memset(data, 0, sizeof(data)); }

    FixedMatrix(int r, int c) : rows(r), cols(c) {
        assert(r <= MAX_N && c <= MAX_N);
        std::memset(data, 0, sizeof(data));
    }

    inline float& operator()(int i, int j) {
        assert(i >= 0 && i < rows && j >= 0 && j < cols);
        return data[i * MAX_N + j];
    }
    inline float operator()(int i, int j) const {
        assert(i >= 0 && i < rows && j >= 0 && j < cols);
        return data[i * MAX_N + j];
    }

    void setZero() {
        for (int i = 0; i < MAX_N * MAX_N; ++i) data[i] = 0.0f;
        rows = cols = 0;
    }

    void resize(int r, int c) {
        assert(r <= MAX_N && c <= MAX_N);
        if (rows == r && cols == c) return;
        // preserve nothing; just set sizes and zero used part
        rows = r; cols = c;
        for (int i = 0; i < rows; ++i)
            for (int j = 0; j < cols; ++j)
                (*this)(i,j) = 0.0f;
    }

    static FixedMatrix Identity(int n) {
        assert(n <= MAX_N);
        FixedMatrix I(n,n);
        for (int i = 0; i < n; ++i) I(i,i) = 1.0f;
        return I;
    }

    // copy assignment
    FixedMatrix& operator=(const FixedMatrix& other) {
        if (this == &other) return *this;
        rows = other.rows; cols = other.cols;
        std::memcpy(data, other.data, sizeof(data));
        return *this;
    }

};

// ベクトルは列ベクトルとして扱う
inline FixedMatrix make_vector(int n) {
    FixedMatrix v(n, 1);
    return v;
}

// 基本演算: C = A * B (A: r x k, B: k x c)
inline void multiply(const FixedMatrix& A, const FixedMatrix& B, FixedMatrix& C) {
    assert(A.cols == B.rows);
    int r = A.rows; int k = A.cols; int c = B.cols;
    C.resize(r, c);
    for (int i = 0; i < r; ++i) {
        for (int j = 0; j < c; ++j) {
            float s = 0.0f;
            for (int t = 0; t < k; ++t) s += A(i,t) * B(t,j);
            C(i,j) = s;
        }
    }
}

inline void add(const FixedMatrix& A, const FixedMatrix& B, FixedMatrix& C) {
    assert(A.rows == B.rows && A.cols == B.cols);
    C.resize(A.rows, A.cols);
    for (int i = 0; i < A.rows; ++i)
        for (int j = 0; j < A.cols; ++j)
            C(i,j) = A(i,j) + B(i,j);
}

inline void sub(const FixedMatrix& A, const FixedMatrix& B, FixedMatrix& C) {
    assert(A.rows == B.rows && A.cols == B.cols);
    C.resize(A.rows, A.cols);
    for (int i = 0; i < A.rows; ++i)
        for (int j = 0; j < A.cols; ++j)
            C(i,j) = A(i,j) - B(i,j);
}

inline void transpose(const FixedMatrix& A, FixedMatrix& At) {
    At.resize(A.cols, A.rows);
    for (int i = 0; i < A.rows; ++i)
        for (int j = 0; j < A.cols; ++j)
            At(j,i) = A(i,j);
}

// Solve S * X = B for X where S is square (n x n), B is (n x m)
// Uses Gaussian elimination with partial pivoting, operates on local copies (stack arrays)
// No heap allocation used; temporary arrays bounded by MAX_N
inline bool solve_linear_system(const FixedMatrix& S, const FixedMatrix& B, FixedMatrix& X) {
    int n = S.rows;
    int m = B.cols;
    assert(S.rows == S.cols);
    assert(B.rows == n);

    // local augmented matrix for each RHS or do elimination once and apply to all RHS
    float A[MAX_N][2*MAX_N]; // we'll create augmented matrix [S | B] with up to n+m <= 2*MAX_N
    if (n > MAX_N || m > MAX_N) return false;

    // initialize augmented
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) A[i][j] = S(i,j);
        for (int j = 0; j < m; ++j) A[i][n + j] = B(i,j);
    }

    // Gaussian elimination with partial pivoting
    for (int col = 0; col < n; ++col) {
        // pivot
        int piv = col;
        float maxv = std::abs(A[col][col]);
        for (int r = col+1; r < n; ++r) {
            float val = std::abs(A[r][col]);
            if (val > maxv) { maxv = val; piv = r; }
        }
        if (maxv < 1e-12f) return false; // singular
        if (piv != col) {
            for (int c = col; c < n + m; ++c) std::swap(A[col][c], A[piv][c]);
        }
        // normalize row
        float diag = A[col][col];
        for (int c = col; c < n + m; ++c) A[col][c] /= diag;
        // eliminate below
        for (int r = 0; r < n; ++r) {
            if (r == col) continue;
            float factor = A[r][col];
            if (factor == 0.0) continue;
            for (int c = col; c < n + m; ++c) A[r][c] -= factor * A[col][c];
        }
    }

    // copy solution
    X.resize(n, m);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            X(i,j) = A[i][n + j];
        }
    }
    return true;
}

// Compute determinant (simple LU-like elimination) and check positive-definiteness
inline bool is_positive_definite(FixedMatrix A) {
    int n = A.rows;
    // attempt Cholesky-like (Doolittle) without allocations
    for (int k = 0; k < n; ++k) {
        if (A(k,k) <= 0.f) return false;
        for (int i = k+1; i < n; ++i) {
            A(i,k) /= A(k,k);
            for (int j = k+1; j < n; ++j) {
                A(i,j) -= A(i,k) * A(k,j);
            }
        }
    }
    return true;
}

} // namespace cmath_fx
