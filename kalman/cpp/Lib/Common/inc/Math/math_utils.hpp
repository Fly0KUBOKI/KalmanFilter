#pragma once

// Implementation: このヘッダー内に実装含む（インライン実装）

#include <cmath>
#include <algorithm>
#include "../../../Matrix/fixed_matrix.hpp"

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
        angle = fmodf(angle + PI, 2.0f * PI);
        if (angle < 0.0f) angle += 2.0f * PI;
        return angle - PI;
    }
    
    // 角度を[-180, 180]に正規化 (度)
    static float wrap_to_180(float angle) {
        angle = fmodf(angle + 180.0f, 360.0f);
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
        for (int i = 0; i < v.rows; ++i) {
            n += v(i,0) * v(i,0);
        }
        n = sqrtf(n);
        
        if (n < EPS) {
            for (int i = 0; i < result.rows; ++i) {
                result(i,0) = 0.0f;
            }
        } else {
            for (int i = 0; i < result.rows; ++i) {
                result(i,0) = v(i,0) / n;
            }
        }
        return result;
    }
    
    // ベクトルのノルム制限
    static cm clip_vector(const cm& v, float max_norm, bool& clipped) {
        cm result = v;
        float n = 0.0f;
        for (int i = 0; i < v.rows; ++i) {
            n += v(i,0) * v(i,0);
        }
        n = sqrtf(n);
        
        if (n > max_norm) {
            float scale = max_norm / n;
            for (int i = 0; i < result.rows; ++i) {
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
        for (int i = 0; i < M.rows; ++i) {
            for (int j = 0; j < M.cols; ++j) {
                result(i,j) = 0.5f * (M(i,j) + M(j,i));
            }
        }
        return result;
    }

    // Innovation とその共分散 S を計算（動的サイズ版）
    // y = z - h
    // S = H * P_pred * H' + R
    static void compute_innovation_and_S(const cm& z, const cm& h, const cm& H,
                                         const cm& P_pred, const cm& R,
                                         cm& y, cm& S, cm& R_out) {
        y = z - h;
        // Compute S
        cm HP = H * P_pred;
        cm HPHt = HP * H.transpose();
        S = HPHt + R;
        S = enforce_symmetry(S);
        R_out = R;
    }

    // 3x3 スキュ対称行列 (ベクトル v に対して)
    static cm skew_symmetric(const cm& v) {
        cm S; S.resize(3,3);
        // v should be 3x1
        float vx = v(0,0);
        float vy = v(1,0);
        float vz = v(2,0);
        S(0,0) = 0.0f;  S(0,1) = -vz;   S(0,2) = vy;
        S(1,0) = vz;    S(1,1) = 0.0f;  S(1,2) = -vx;
        S(2,0) = -vy;   S(2,1) = vx;    S(2,2) = 0.0f;
        return S;
    }
    
    // ========== 数値安定化 ==========
    
    // 安全な除算（ゼロ除算回避）
    static float safe_divide(float numerator, float denominator, float default_value = 0.0f) {
        if (fabsf(denominator) < EPS) {
            return default_value;
        }
        return numerator / denominator;
    }
    
    // 安全な平方根（負数回避）
    static float safe_sqrt(float x) {
        return sqrtf(fmaxf(x, 0.0f));
    }
    
    // 安全なasin（定義域制限）
    static float safe_asin(float x) {
        x = fmaxf(fminf(x, 1.0f), -1.0f);
        return asinf(x);
    }
    
    // 安全なacos（定義域制限）
    static float safe_acos(float x) {
        x = fmaxf(fminf(x, 1.0f), -1.0f);
        return acosf(x);
    }
    
    // ========== 統計 ==========
    
    // 中央値
    static float median(const cm& data) {
        if (data.rows == 0) return 0.0f;
        
        // データをコピーしてソート（固定配列を使用）
        const int MAX_SIZE = 20;  // FixedMatrix::MAX_Nと同じ
        int n = data.rows;
        if (n > MAX_SIZE) n = MAX_SIZE;  // 安全のため
        
        float sorted[MAX_SIZE];
        for (int i = 0; i < n; ++i) {
            sorted[i] = data(i,0);
        }
        std::sort(sorted, sorted + n);
        
        float result;
        if (n % 2 == 0) {
            result = (sorted[n/2-1] + sorted[n/2]) * 0.5f;
        } else {
            result = sorted[n/2];
        }
        
        return result;
    }
    
    // MAD (Median Absolute Deviation)
    static float mad(const cm& data) {
        if (data.rows == 0) return 0.0f;
        
        float med = median(data);
        
        cm abs_dev;
        abs_dev.resize(data.rows, 1);
        for (int i = 0; i < data.rows; ++i) {
            abs_dev(i,0) = fabsf(data(i,0) - med);
        }
        
        return median(abs_dev);
    }
    
    // ロバストな統計（外れ値除外）
    static void robust_statistics(const cm& data, float& mean_val, float& std_val, 
                                  float outlier_threshold = 3.0f) {
        if (data.rows == 0) {
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
        
        for (int i = 0; i < data.rows; ++i) {
            float z_score = fabsf(data(i,0) - mu) / (sigma + EPS);
            if (z_score < outlier_threshold) {
                sum += data(i,0);
                sum_sq += data(i,0) * data(i,0);
                n_inliers++;
            }
        }
        
        if (n_inliers > 0) {
            mean_val = sum / n_inliers;
            std_val = sqrtf(sum_sq / n_inliers - mean_val * mean_val);
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
        float cos_lat0 = cosf(lat0 * PI / 180.0f);
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
        float cos_lat0 = cosf(lat0 * PI / 180.0f);
        float m_to_deg_lon = 9.0e-6f / cos_lat0;
        
        lon = lon0 + x_enu * m_to_deg_lon;
        lat = lat0 + y_enu * m_to_deg_lat;
        alt = alt0 + z_enu;
    }
    
    // ========== 補間 ==========
    
    // 線形補間
    static float linear_interpolate(float x, float x1, float y1, float x2, float y2) {
        if (fabsf(x2 - x1) < EPS) {
            return y1;
        }
        float t = (x - x1) / (x2 - x1);
        return y1 + t * (y2 - y1);
    }
    
    // ========== 行列分解 ==========
    
    // 3x3行列の逆行列計算（ガウス消去法）
    // A: 入力行列 (3x3)
    // A_inv: 逆行列 (3x3) [出力]
    // 戻り値: 成功時true、特異行列の場合はfalse
    template<typename T>
    static bool invert3x3(const cmath_fx::Matrix<3, 3, T>& A, cmath_fx::Matrix<3, 3, T>& A_inv) {
        // 行列式を計算
        T det = A(0,0) * (A(1,1)*A(2,2) - A(2,1)*A(1,2))
              - A(0,1) * (A(1,0)*A(2,2) - A(2,0)*A(1,2))
              + A(0,2) * (A(1,0)*A(2,1) - A(2,0)*A(1,1));
        
        if (std::abs(det) < static_cast<T>(1e-10)) {
            return false;
        }
        
        T inv_det = static_cast<T>(1.0) / det;
        
        // 随伴行列を計算
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
    
    // 安全なCholesky分解（正定値でない場合は正則化）
    static bool safe_cholesky(const cm& A, cm& L) {
        if (A.rows != A.cols) return false;
        
        int n = A.rows;
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
                    L(i,j) = sqrtf(diag);
                } else {
                    if (fabsf(L(j,j)) < EPS) {
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
    
    // ========== 異常検出・距離計算 ==========
    
    // Mahalanobis距離を計算する統一関数
    // マハラノビス距離^2 = (y - μ)' * Σ^-1 * (y - μ)
    // ここで y = innovation, Σ = innovation共分散行列 S
    // 簡易版: S^-1を直接計算せず、Cholesky分解を使用
    static float mahalanobis_distance_squared(const cm& innovation, const cm& S) {
        int n = innovation.rows;
        if (n == 0) return 0.0f;
        
        // S を lower-triangular で Cholesky分解: S = L * L'
        cm L(n, n);
        if (!safe_cholesky(S, L)) {
            // 分解失敗→対角成分の最大値で正規化（フォールバック）
            float max_var = 0.0f;
            for (int i = 0; i < n; ++i) {
                if (S(i,i) > max_var) max_var = S(i,i);
            }
            if (max_var < EPS) max_var = 1.0f;
            
            float norm_sq = 0.0f;
            for (int i = 0; i < n; ++i) {
                norm_sq += innovation(i,0) * innovation(i,0);
            }
            return norm_sq / max_var;
        }
        
        // L の逆行列を求める（前進代入）
        cm L_inv(n, n);
        for (int j = 0; j < n; ++j) {
            for (int i = 0; i < n; ++i) {
                if (i < j) {
                    L_inv(i,j) = 0.0f;
                    continue;
                }
                
                float sum = 0.0f;
                for (int k = 0; k < i; ++k) {
                    sum += L(i,k) * L_inv(k,j);
                }
                
                if (i == j) {
                    if (fabsf(L(i,i)) < EPS) {
                        L_inv(i,j) = 1.0f / EPS;
                    } else {
                        L_inv(i,j) = (1.0f - sum) / L(i,i);
                    }
                } else {
                    L_inv(i,j) = -sum / L(i,i);
                }
            }
        }
        
        // y' = (L^-1) * innovation
        cm L_inv_y(n, 1);
        for (int i = 0; i < n; ++i) {
            float sum = 0.0f;
            for (int j = 0; j < n; ++j) {
                sum += L_inv(i,j) * innovation(j,0);
            }
            L_inv_y(i,0) = sum;
        }
        
        // マハラノビス距離^2 = |L^-1 * y|^2
        float dist_sq = 0.0f;
        for (int i = 0; i < n; ++i) {
            dist_sq += L_inv_y(i,0) * L_inv_y(i,0);
        }
        
        return dist_sq;
    }
    
    // Mahalanobis距離（スカラー版）
    static float mahalanobis_distance_scalar(float innovation, float variance) {
        if (variance < EPS) variance = EPS;
        return fabsf(innovation) / sqrtf(variance);
    }
    
    // Chi-square検定による外れ値判定
    // dof = 自由度（innovation の次元）
    static bool is_outlier_chi_square(const cm& innovation, const cm& S, int dof, float alpha = 0.05f) {
        // 自由度dof、有意水準αのχ²分布の臨界値（近似）
        // α=0.05の場合: χ²_{1,0.05}≈3.84, χ²_{3,0.05}≈7.81
        float chi_sq_critical = 3.0f * dof;  // 保守的な閾値
        
        float dist_sq = mahalanobis_distance_squared(innovation, S);
        return (dist_sq > chi_sq_critical);
    }
};

} // namespace math
} // namespace common
