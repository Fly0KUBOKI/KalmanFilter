#pragma once

#ifndef MEX_MEX_HELPERS_HPP
#define MEX_MEX_HELPERS_HPP

#include <mex.h>
#include <string>
#include <cstring>
#include "../../Lib/Quaternion/quaternion_functions.hpp"
#include "../../Lib/Matrix/fixed_matrix.hpp"

namespace mex_helpers {

/**
 * MEX用ヘルパー関数群
 * MATLAB配列の操作やコマンド取得などの共通処理
 */

/**
 * MATLAB配列から文字列コマンドを取得
 * @param a mxArrayポインタ
 * @return コマンド文字列（空文字列の場合は失敗）
 */
inline std::string getCmd(const mxArray* a) {
    char buf[256] = {0};
    if (!mxIsChar(a)) return "";
    mxGetString(a, buf, sizeof(buf));
    return std::string(buf);
}

/**
 * MATLAB構造体から3次元ベクトルを取得（single型のみ）
 * GPS以外のセンサーデータはMATLAB側でsingle型で渡す必要がある
 * @param s 構造体mxArrayポインタ
 * @param xname X成分のフィールド名
 * @param yname Y成分のフィールド名
 * @param zname Z成分のフィールド名
 * @param idx 配列インデックス（0-based）
 * @param out 出力配列（3要素、double）
 */
inline void getVec3(const mxArray* s, const char* xname, const char* yname, const char* zname, mwIndex idx, double* out) {
    mxArray* fx = mxGetField(s, 0, xname);
    mxArray* fy = mxGetField(s, 0, yname);
    mxArray* fz = mxGetField(s, 0, zname);
    
    // single型のみを受け取る（MATLAB側でsingle型で渡す必要がある）
    auto get_value = [](const mxArray* arr, mwIndex i, const char* name) -> double {
        if (!arr) return 0.0;
        if (mxGetClassID(arr) != mxSINGLE_CLASS) {
            mexErrMsgIdAndTxt("mex_helpers:type_error", 
                "Expected single (float) array for field '%s', but got %s.", 
                name, mxGetClassName(arr));
            return 0.0;
        }
        const float* pf = (const float*)mxGetData(arr);
        return static_cast<double>(pf[i]);
    };
    
    out[0] = get_value(fx, idx, xname);
    out[1] = get_value(fy, idx, yname);
    out[2] = get_value(fz, idx, zname);
}

/**
 * 配列のコピー（double型）
 * @param dst コピー先
 * @param src コピー元
 * @param n 要素数
 */
inline void copy_vec(double* dst, const double* src, int n) {
    memcpy(dst, src, n * sizeof(double));
}

/**
 * 配列にNaNが含まれているかチェック
 * @param v チェック対象配列
 * @param n 要素数
 * @return NaNが含まれていればtrue
 */
inline bool is_nan_any(const double* v, int n) {
    for (int i = 0; i < n; ++i) {
        if (mxIsNaN(v[i])) return true;
    }
    return false;
}

/**
 * MATLAB構造体からフィールドを取得
 * @param s 構造体mxArrayポインタ
 * @param name フィールド名
 * @return フィールドのmxArrayポインタ（存在しない場合はnullptr）
 */
inline const mxArray* get_field(const mxArray* s, const char* name) {
    if (!mxIsStruct(s)) return nullptr;
    return mxGetField(s, 0, name);
}

/**
 * MATLAB構造体からフィールドを取得（複数候補）
 * @param s 構造体mxArrayポインタ
 * @param name1 第1候補のフィールド名
 * @param name2 第2候補のフィールド名
 * @return フィールドのmxArrayポインタ（存在しない場合はnullptr）
 */
inline const mxArray* get_field_any(const mxArray* s, const char* name1, const char* name2) {
    const mxArray* f = get_field(s, name1);
    if (f) return f;
    return get_field(s, name2);
}

/**
 * MATLAB配列からデータポインタを取得
 * @param arr mxArrayポインタ
 * @return データポインタ（存在しない場合はnullptr）
 */
inline double* get_data(const mxArray* arr) {
    if (!arr) return nullptr;
    return mxGetPr(arr);
}

/**
 * MATLAB配列の要素数を取得
 * @param arr mxArrayポインタ
 * @return 要素数
 */
inline int get_length(const mxArray* arr) {
    if (!arr) return 0;
    return static_cast<int>(mxGetNumberOfElements(arr));
}

/**
 * クォータニオンをオイラー角（ラジアン）に変換
 * @param q_in クォータニオン配列 [w, x, y, z]
 * @param euler 出力オイラー角配列 [roll, pitch, yaw] (ラジアン)
 */
inline void quat_to_euler(const double* q_in, double* euler) {
    cmath_fx::Vector<4, float> q;
    q(0,0) = static_cast<float>(q_in[0]);
    q(1,0) = static_cast<float>(q_in[1]);
    q(2,0) = static_cast<float>(q_in[2]);
    q(3,0) = static_cast<float>(q_in[3]);
    cquat::normalize_quat(q);
    
    float roll_deg, pitch_deg, yaw_deg;
    cquat::to_euler_deg(q, roll_deg, pitch_deg, yaw_deg);
    
    // Convert degrees to radians
    const double DEG2RAD = 3.14159265358979323846 / 180.0;
    euler[0] = static_cast<double>(roll_deg) * DEG2RAD;
    euler[1] = static_cast<double>(pitch_deg) * DEG2RAD;
    euler[2] = static_cast<double>(yaw_deg) * DEG2RAD;
}

} // namespace mex_helpers

#endif // MEX_MEX_HELPERS_HPP

