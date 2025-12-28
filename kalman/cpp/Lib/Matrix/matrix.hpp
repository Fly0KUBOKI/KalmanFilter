#pragma once

#include <cmath>
#include <cstring>
#include "../Common/types.hpp"

namespace lib {
namespace matrix {

// 固定サイズ行列 (コンパイル時サイズ決定)
template<Index R, Index C, typename T = Scalar>
struct Mat {
    static constexpr Index Rows = R;
    static constexpr Index Cols = C;
    T data[R * C];
    
    // コンストラクタ (ゼロ初期化)
    Mat() {
        std::memset(data, 0, sizeof(data));
    }
    
    // 要素アクセス
    T& operator()(Index r, Index c) {
        return data[r * C + c];
    }
    
    const T& operator()(Index r, Index c) const {
        return data[r * C + c];
    }
    
    // 静的ファクトリ
    static Mat Zero() {
        return Mat();
    }
    
    static Mat Identity() {
        static_assert(R == C, "Identity matrix must be square");
        Mat m;
        for (Index i = 0; i < R; ++i) {
            m(i, i) = static_cast<T>(1);
        }
        return m;
    }
    
    // 加算
    Mat operator+(const Mat& other) const {
        Mat res;
        for (Index i = 0; i < R * C; ++i) {
            res.data[i] = data[i] + other.data[i];
        }
        return res;
    }
    
    // 減算
    Mat operator-(const Mat& other) const {
        Mat res;
        for (Index i = 0; i < R * C; ++i) {
            res.data[i] = data[i] - other.data[i];
        }
        return res;
    }
    
    // スカラー倍
    Mat operator*(T scalar) const {
        Mat res;
        for (Index i = 0; i < R * C; ++i) {
            res.data[i] = data[i] * scalar;
        }
        return res;
    }
    
    // 行列積
    template<Index K>
    Mat<R, K, T> operator*(const Mat<C, K, T>& other) const {
        Mat<R, K, T> res;
        for (Index i = 0; i < R; ++i) {
            for (Index j = 0; j < K; ++j) {
                T sum = static_cast<T>(0);
                for (Index k = 0; k < C; ++k) {
                    sum += (*this)(i, k) * other(k, j);
                }
                res(i, j) = sum;
            }
        }
        return res;
    }
    
    // 転置
    Mat<C, R, T> transpose() const {
        Mat<C, R, T> res;
        for (Index i = 0; i < R; ++i) {
            for (Index j = 0; j < C; ++j) {
                res(j, i) = (*this)(i, j);
            }
        }
        return res;
    }
    
    // 逆行列 (Gauss-Jordan) - 正方行列のみ
    bool inverse(Mat& inv) const {
        static_assert(R == C, "Inverse requires square matrix");
        
        Mat<R, R * 2, T> aug;
        
        // 拡大係数行列 [A | I]
        for (Index i = 0; i < R; ++i) {
            for (Index j = 0; j < R; ++j) {
                aug(i, j) = (*this)(i, j);
            }
            aug(i, R + i) = static_cast<T>(1);
        }
        
        for (Index i = 0; i < R; ++i) {
            // ピボット選択
            Index pivot = i;
            T max_val = std::abs(aug(i, i));
            
            for (Index k = i + 1; k < R; ++k) {
                if (std::abs(aug(k, i)) > max_val) {
                    max_val = std::abs(aug(k, i));
                    pivot = k;
                }
            }
            
            if (max_val < static_cast<T>(1e-12f)) {
                return false; // 特異行列
            }
            
            // 行入れ替え
            if (pivot != i) {
                for (Index j = 0; j < 2 * R; ++j) {
                    T tmp = aug(i, j);
                    aug(i, j) = aug(pivot, j);
                    aug(pivot, j) = tmp;
                }
            }
            
            // 正規化
            T div = aug(i, i);
            for (Index j = i; j < 2 * R; ++j) {
                aug(i, j) /= div;
            }
            
            // 消去
            for (Index k = 0; k < R; ++k) {
                if (k != i) {
                    T factor = aug(k, i);
                    for (Index j = i; j < 2 * R; ++j) {
                        aug(k, j) -= factor * aug(i, j);
                    }
                }
            }
        }
        
        // 結果の取り出し
        for (Index i = 0; i < R; ++i) {
            for (Index j = 0; j < R; ++j) {
                inv(i, j) = aug(i, R + j);
            }
        }
        
        return true;
    }
};

// ベクトル型エイリアス
template<Index N, typename T = Scalar>
using Vec = Mat<N, 1, T>;

// よく使うサイズ
using Vec3 = Vec<3>;
using Vec4 = Vec<4>;
using Mat3 = Mat<3, 3>;
using Mat4 = Mat<4, 4>;

} // namespace matrix
} // namespace lib

