#pragma once

// Implementation: このヘッダー内に実装含む（テンプレート実装）

#include <cmath>
#include <cstring>
#include <cassert>
#include <algorithm>
#include <iostream>

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

        for(int i=0; i<R; ++i) {

            for(int j=0; j<R; ++j) aug(i, j) = (*this)(i, j);

            aug(i, R+i) = static_cast<T>(1);

        }



        for(int i=0; i<R; ++i) {

            // ピボット選択

            int pivot = i;

            T max_val = std::abs(aug(i, i));

            for(int k=i+1; k<R; ++k) {

                if(std::abs(aug(k, i)) > max_val) {

                    max_val = std::abs(aug(k, i));

                    pivot = k;

                }

            }

            

            if(max_val < static_cast<T>(1e-12)) return false; // 特異行列



            // 行入れ替え

            if(pivot != i) {

                for(int j=0; j<2*R; ++j) std::swap(aug(i, j), aug(pivot, j));

            }



            // 正規化

            T div = aug(i, i);

            for(int j=i; j<2*R; ++j) aug(i, j) /= div;



            // 消去

            for(int k=0; k<R; ++k) {

                if(k != i) {

                    T factor = aug(k, i);

                    for(int j=i; j<2*R; ++j) aug(k, j) -= factor * aug(i, j);

                }

            }

        }



        // 結果の取り出し

        for(int i=0; i<R; ++i) {

            for(int j=0; j<R; ++j) inv(i, j) = aug(i, R+j);

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

        for(int i=0; i<R; ++i)

            for(int j=0; j<C; ++j)

                m(i,j) = (*this)(i,j);

        return m;

    }



    // Assignment from Template Matrix

    template <int R, int C>

    void fromMatrix(const Matrix<R, C, float>& m) {

        rows = R;

        cols = C;

        for(int i=0; i<R; ++i)

            for(int j=0; j<C; ++j)

                (*this)(i,j) = m(i,j);

    }

    

    // Basic operations needed for MEX

    FixedMatrix transpose() const {

        FixedMatrix res(cols, rows);

        for(int i=0; i<rows; ++i)

            for(int j=0; j<cols; ++j)

                res(j,i) = (*this)(i,j);

        return res;

    }

    

    FixedMatrix operator+(const FixedMatrix& other) const {

        assert(rows == other.rows && cols == other.cols);

        FixedMatrix res(rows, cols);

        for(int i=0; i<rows*cols; ++i) res.data[i] = data[i] + other.data[i];

        return res;

    }

    

    FixedMatrix operator-(const FixedMatrix& other) const {

        assert(rows == other.rows && cols == other.cols);

        FixedMatrix res(rows, cols);

        for(int i=0; i<rows*cols; ++i) res.data[i] = data[i] - other.data[i];

        return res;

    }

    

    FixedMatrix operator*(const FixedMatrix& other) const {

        assert(cols == other.rows);

        FixedMatrix res(rows, other.cols);

        for(int i=0; i<rows; ++i) {

            for(int j=0; j<other.cols; ++j) {

                float sum = 0;

                for(int k=0; k<cols; ++k) {

                    sum += (*this)(i,k) * other(k,j);

                }

                res(i,j) = sum;

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

        for(int i=0; i<n; ++i) {

            for(int j=0; j<n; ++j) aug[i][j] = (*this)(i,j);

            for(int j=n; j<2*n; ++j) aug[i][j] = (j-n == i) ? 1.0f : 0.0f;

        }

        

        for(int i=0; i<n; ++i) {

            int pivot = i;

            float max_val = std::abs(aug[i][i]);

            for(int k=i+1; k<n; ++k) {

                if(std::abs(aug[k][i]) > max_val) {

                    max_val = std::abs(aug[k][i]);

                    pivot = k;

                }

            }

            if(max_val < 1e-12f) return false;

            

            if(pivot != i) {

                for(int j=0; j<2*n; ++j) std::swap(aug[i][j], aug[pivot][j]);

            }

            

            float div = aug[i][i];

            for(int j=i; j<2*n; ++j) aug[i][j] /= div;

            

            for(int k=0; k<n; ++k) {

                if(k != i) {

                    float factor = aug[k][i];

                    for(int j=i; j<2*n; ++j) aug[k][j] -= factor * aug[i][j];

                }

            }

        }

        

        for(int i=0; i<n; ++i)

            for(int j=0; j<n; ++j)

                inv(i,j) = aug[i][n+j];

                

        return true;

    }

};



}
