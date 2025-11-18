#pragma once

#include <cmath>
#include <algorithm>
#include "../Math/fixed_matrix.hpp"

namespace common {
namespace math {

using cm = cmath_fx::FixedMatrix;

class MathUtils {
public:
    static constexpr float EPS = 1.0e-9f;
    static constexpr float PI = 3.14159265358979323846f;
    
    // ========== 角度処理 ==========
    
    // 角度を[-π, π]に正規化 (rad)
    static float wrap_to_pi(float angle) {
        angle = std::fmodf(angle + PI, 2.0f * PI);
        if (angle < 0.0f) angle += 2.0f * PI;
        return angle - PI;
    }
    
    // 角度を[-180, 180]に正規化 (度)
    static float wrap_to_180(float angle) {
        angle = std::fmodf(angle + 180.0f, 360.0f);
        if (angle < 0.0f) angle += 360.0f;
        return angle - 180.0f;
    }
    
    // 角度差を[-π, π]で計算 (rad)
    static float angle_difference(float a1, float a2) {
        return wrap_to_pi(a2 - a1);
    }
    
    // ========== ベクトル・行列操作 ==========
    
    // ベクトル正規化
    static cm normalize_vector(const cm& v) {
        cm result = v;
        float n = 0.0f;
        for (int i = 0; i < v.rows(); ++i) {
            n += v(i,0) * v(i,0);
        }
        n = std::sqrtf(n);
        
        if (n < EPS) {
            for (int i = 0; i < result.rows(); ++i) {
                result(i,0) = 0.0f;
            }
        } else {
            for (int i = 0; i < result.rows(); ++i) {
                result(i,0) = v(i,0) / n;
            }
        }
        return result;
    }
    
    // ベクトルのノルム制限
    static cm clip_vector(const cm& v, float max_norm, bool& clipped) {
        cm result = v;
        float n = 0.0f;
        for (int i = 0; i < v.rows(); ++i) {
            n += v(i,0) * v(i,0);
        }
        n = std::sqrtf(n);
        
        if (n > max_norm) {
            float scale = max_norm / n;
            for (int i = 0; i < result.rows(); ++i) {
                result(i,0) = v(i,0) * scale;
            }
            clipped = true;
        } else {
            clipped = false;
        }
        return result;
    }
    
    // 行列の対称性を強制
    static cm enforce_symmetry(const cm& M) {
        cm result = M;
        for (int i = 0; i < M.rows(); ++i) {
            for (int j = 0; j < M.cols(); ++j) {
                result(i,j) = 0.5f * (M(i,j) + M(j,i));
            }
        }
        return result;
    }
    
    // ========== 数値安定化 ==========
    
    // 安全な除算（ゼロ除算回避）
    static float safe_divide(float numerator, float denominator, float default_value = 0.0f) {
        if (std::fabsf(denominator) < EPS) {
            return default_value;
        }
        return numerator / denominator;
    }
    
    // 安全な平方根（負数回避）
    static float safe_sqrt(float x) {
        return std::sqrtf(std::fmaxf(x, 0.0f));
    }
    
    // 安全なasin（定義域制限）
    static float safe_asin(float x) {
        x = std::fmaxf(std::fminf(x, 1.0f), -1.0f);
        return std::asinf(x);
    }
    
    // 安全なacos（定義域制限）
    static float safe_acos(float x) {
        x = std::fmaxf(std::fminf(x, 1.0f), -1.0f);
        return std::acosf(x);
    }
    
    // ========== 統計 ==========
    
    // 中央値
    static float median(const cm& data) {
        if (data.rows() == 0) return 0.0f;
        
        // データをコピーしてソート
        int n = data.rows();
        float* sorted = new float[n];
        for (int i = 0; i < n; ++i) {
            sorted[i] = data(i,0);
        }
        std::sort(sorted, sorted + n);
        
        float result;
        if (n % 2 == 0) {
            result = (sorted[n/2-1] + sorted[n/2]) / 2.0f;
        } else {
            result = sorted[n/2];
        }
        
        delete[] sorted;
        return result;
    }
    
    // MAD (Median Absolute Deviation)
    static float mad(const cm& data) {
        if (data.rows() == 0) return 0.0f;
        
        float med = median(data);
        
        cm abs_dev;
        abs_dev.resize(data.rows(), 1);
        for (int i = 0; i < data.rows(); ++i) {
            abs_dev(i,0) = std::fabsf(data(i,0) - med);
        }
        
        return median(abs_dev);
    }
    
    // ロバストな統計（外れ値除外）
    static void robust_statistics(const cm& data, float& mean_val, float& std_val, 
                                  float outlier_threshold = 3.0f) {
        if (data.rows() == 0) {
            mean_val = 0.0f;
            std_val = 0.0f;
            return;
        }
        
        // 初期推定
        float mu = median(data);
        float sigma = 1.4826f * mad(data);
        
        // 外れ値除外
        int n_inliers = 0;
        float sum = 0.0f;
        float sum_sq = 0.0f;
        
        for (int i = 0; i < data.rows(); ++i) {
            float z_score = std::fabsf(data(i,0) - mu) / (sigma + EPS);
            if (z_score < outlier_threshold) {
                sum += data(i,0);
                sum_sq += data(i,0) * data(i,0);
                n_inliers++;
            }
        }
        
        if (n_inliers > 0) {
            mean_val = sum / n_inliers;
            std_val = std::sqrtf(sum_sq / n_inliers - mean_val * mean_val);
        } else {
            mean_val = mu;
            std_val = sigma;
        }
    }
    
    // ========== 座標変換 ==========
    
    // 緯度経度高度からENU座標への変換（簡易版・球面近似）
    static void lla_to_enu(float lat, float lon, float alt,
                          float lat0, float lon0, float alt0,
                          float& x_enu, float& y_enu, float& z_enu) {
        // 度からメートルへの変換係数
        float deg_to_m_lat = 1.0f / 9.0e-6f;
        float cos_lat0 = std::cosf(lat0 * PI / 180.0f);
        float deg_to_m_lon = 1.0f / (9.0e-6f / cos_lat0);
        
        x_enu = (lon - lon0) * deg_to_m_lon;  // East
        y_enu = (lat - lat0) * deg_to_m_lat;  // North
        z_enu = alt - alt0;                    // Up
    }
    
    // ENU座標から緯度経度高度への変換（簡易版・球面近似）
    static void enu_to_lla(float x_enu, float y_enu, float z_enu,
                          float lat0, float lon0, float alt0,
                          float& lat, float& lon, float& alt) {
        // メートルから度への変換係数
        float m_to_deg_lat = 9.0e-6f;
        float cos_lat0 = std::cosf(lat0 * PI / 180.0f);
        float m_to_deg_lon = 9.0e-6f / cos_lat0;
        
        lon = lon0 + x_enu * m_to_deg_lon;
        lat = lat0 + y_enu * m_to_deg_lat;
        alt = alt0 + z_enu;
    }
    
    // ========== 補間 ==========
    
    // 線形補間
    static float linear_interpolate(float x, float x1, float y1, float x2, float y2) {
        if (std::fabsf(x2 - x1) < EPS) {
            return y1;
        }
        float t = (x - x1) / (x2 - x1);
        return y1 + t * (y2 - y1);
    }
    
    // ========== 行列分解 ==========
    
    // 安全なCholesky分解（正定値でない場合は正則化）
    static bool safe_cholesky(const cm& A, cm& L) {
        if (A.rows() != A.cols()) return false;
        
        int n = A.rows();
        L.resize(n, n);
        
        // Cholesky分解を試行
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j <= i; ++j) {
                float sum = 0.0f;
                for (int k = 0; k < j; ++k) {
                    sum += L(i,k) * L(j,k);
                }
                
                if (i == j) {
                    float diag = A(i,i) - sum;
                    if (diag <= 0.0f) {
                        // 正定値でない→正則化して単位行列
                        for (int ii = 0; ii < n; ++ii) {
                            for (int jj = 0; jj < n; ++jj) {
                                L(ii,jj) = (ii == jj) ? 1.0f : 0.0f;
                            }
                        }
                        return false;
                    }
                    L(i,j) = std::sqrtf(diag);
                } else {
                    if (std::fabsf(L(j,j)) < EPS) {
                        L(i,j) = 0.0f;
                    } else {
                        L(i,j) = (A(i,j) - sum) / L(j,j);
                    }
                }
            }
            
            // 上三角を0に
            for (int j = i+1; j < n; ++j) {
                L(i,j) = 0.0f;
            }
        }
        
        return true;
    }
};

} // namespace math
} // namespace common
