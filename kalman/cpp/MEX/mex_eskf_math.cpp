// mex_eskf_math.cpp
// MATLAB MEX wrapper for ESKF math library (stateless computation functions)
//
// Usage examples:
//   q_new = mex_eskf_math('quaternion_integration', q, w, dt);
//   [p_new, v_new] = mex_eskf_math('pv_integration', p, v, a_world, g, dt, prev_a, prev_v, use_ab2, max_accel, max_vel);
//   F = mex_eskf_math('compute_F_matrix', q, a_meas, ba, w_meas, bg, dt);
//   P_new = mex_eskf_math('covariance_prediction', P, F, Q);
//   [p, v, q, ba, bg] = mex_eskf_math('inject_error_state', p, v, q, ba, bg, dx);
//   [x_new, P_new, K, S] = mex_eskf_math('kalman_update', x, P, y, H, R);

#include "mex.h"
#include "../include/ESKF/eskf_math.hpp"
#include <string>
#include <cstring>

using namespace eskf_math;

// Helper: MATLAB array -> Vector
template<int N>
static bool matToVector(const mxArray* arr, cmath_fx::Vector<N, Scalar>& out) {
    if (!mxIsDouble(arr) || mxIsComplex(arr)) return false;
    mwSize numel = mxGetNumberOfElements(arr);
    if (numel != N) return false;
    double* pr = mxGetPr(arr);
    for (int i = 0; i < N; ++i) {
        out(i, 0) = static_cast<Scalar>(pr[i]);
    }
    return true;
}

// Helper: Vector -> MATLAB array
template<int N>
static mxArray* vectorToMat(const cmath_fx::Vector<N, Scalar>& v) {
    mxArray* out = mxCreateDoubleMatrix(N, 1, mxREAL);
    double* pr = mxGetPr(out);
    for (int i = 0; i < N; ++i) {
        pr[i] = static_cast<double>(v(i, 0));
    }
    return out;
}

// Helper: MATLAB array -> Matrix
template<int R, int C>
static bool matToMatrix(const mxArray* arr, cmath_fx::Matrix<R, C, Scalar>& out) {
    if (!mxIsDouble(arr) || mxIsComplex(arr)) return false;
    mwSize rows = mxGetM(arr);
    mwSize cols = mxGetN(arr);
    if (rows != R || cols != C) return false;
    double* pr = mxGetPr(arr);
    for (int j = 0; j < C; ++j) {
        for (int i = 0; i < R; ++i) {
            out(i, j) = static_cast<Scalar>(pr[j * rows + i]);
        }
    }
    return true;
}

// Helper: Matrix -> MATLAB array
template<int R, int C>
static mxArray* matrixToMat(const cmath_fx::Matrix<R, C, Scalar>& M) {
    mxArray* out = mxCreateDoubleMatrix(R, C, mxREAL);
    double* pr = mxGetPr(out);
    for (int j = 0; j < C; ++j) {
        for (int i = 0; i < R; ++i) {
            pr[j * R + i] = static_cast<double>(M(i, j));
        }
    }
    return out;
}

// Helper: Get scalar from MATLAB array
static Scalar getScalar(const mxArray* arr) {
    if (!mxIsDouble(arr) || mxGetNumberOfElements(arr) != 1) {
        mexErrMsgTxt("Expected scalar double");
    }
    return static_cast<Scalar>(mxGetScalar(arr));
}

// Handler: quaternion_integration
// q_new = mex_eskf_math('quaternion_integration', q, w, dt)
static void handle_quaternion_integration(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 4) mexErrMsgTxt("quaternion_integration: Expected 3 args (q, w, dt)");
    if (nlhs > 1) mexErrMsgTxt("quaternion_integration: Expected 1 output");
    
    Vector4 q_in, q_out;
    Vector3 w;
    if (!matToVector<4>(prhs[1], q_in)) mexErrMsgTxt("q must be 4x1");
    if (!matToVector<3>(prhs[2], w)) mexErrMsgTxt("w must be 3x1");
    Scalar dt = getScalar(prhs[3]);
    
    ESKFMath::quaternion_integration(q_in, w, dt, q_out);
    plhs[0] = vectorToMat<4>(q_out);
}

// Handler: accel_to_quaternion
// q = mex_eskf_math('accel_to_quaternion', a_meas, scale_factor)
static void handle_accel_to_quaternion(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 3) mexErrMsgTxt("accel_to_quaternion: Expected 2 args (a_meas, scale)");
    if (nlhs > 1) mexErrMsgTxt("accel_to_quaternion: Expected 1 output");
    
    Vector3 a_meas;
    Vector4 q_out;
    if (!matToVector<3>(prhs[1], a_meas)) mexErrMsgTxt("a_meas must be 3x1");
    Scalar scale = getScalar(prhs[2]);
    
    ESKFMath::accel_to_quaternion(a_meas, scale, q_out);
    plhs[0] = vectorToMat<4>(q_out);
}

// Handler: pv_integration
// [p_new, v_new] = mex_eskf_math('pv_integration', p, v, a_world, g, dt, prev_a, prev_v, use_ab2, max_accel, max_vel)
static void handle_pv_integration(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 11) mexErrMsgTxt("pv_integration: Expected 10 args");
    if (nlhs > 2) mexErrMsgTxt("pv_integration: Expected 2 outputs");
    
    ESKFMath::PVIntegrationInput input;
    ESKFMath::PVIntegrationOutput output;
    
    if (!matToVector<3>(prhs[1], input.p)) mexErrMsgTxt("p must be 3x1");
    if (!matToVector<3>(prhs[2], input.v)) mexErrMsgTxt("v must be 3x1");
    if (!matToVector<3>(prhs[3], input.a_world)) mexErrMsgTxt("a_world must be 3x1");
    if (!matToVector<3>(prhs[4], input.g)) mexErrMsgTxt("g must be 3x1");
    input.dt = getScalar(prhs[5]);
    if (!matToVector<3>(prhs[6], input.prev_a)) mexErrMsgTxt("prev_a must be 3x1");
    if (!matToVector<3>(prhs[7], input.prev_v)) mexErrMsgTxt("prev_v must be 3x1");
    input.use_ab2 = getScalar(prhs[8]) > 0.5;
    input.max_accel = getScalar(prhs[9]);
    input.max_velocity = getScalar(prhs[10]);
    
    ESKFMath::pv_integration(input, output);
    
    plhs[0] = vectorToMat<3>(output.p_new);
    if (nlhs > 1) plhs[1] = vectorToMat<3>(output.v_new);
}

// Handler: compute_F_matrix
// F = mex_eskf_math('compute_F_matrix', q, a_meas, ba, w_meas, bg, dt)
static void handle_compute_F_matrix(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 7) mexErrMsgTxt("compute_F_matrix: Expected 6 args");
    if (nlhs > 1) mexErrMsgTxt("compute_F_matrix: Expected 1 output");
    
    Vector4 q;
    Vector3 a_meas, ba, w_meas, bg;
    Matrix15x15 F;
    
    if (!matToVector<4>(prhs[1], q)) mexErrMsgTxt("q must be 4x1");
    if (!matToVector<3>(prhs[2], a_meas)) mexErrMsgTxt("a_meas must be 3x1");
    if (!matToVector<3>(prhs[3], ba)) mexErrMsgTxt("ba must be 3x1");
    if (!matToVector<3>(prhs[4], w_meas)) mexErrMsgTxt("w_meas must be 3x1");
    if (!matToVector<3>(prhs[5], bg)) mexErrMsgTxt("bg must be 3x1");
    Scalar dt = getScalar(prhs[6]);
    
    ESKFMath::compute_F_matrix(q, a_meas, ba, w_meas, bg, dt, F);
    plhs[0] = matrixToMat<15, 15>(F);
}

// Handler: covariance_prediction
// P_new = mex_eskf_math('covariance_prediction', P, F, Q)
static void handle_covariance_prediction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 4) mexErrMsgTxt("covariance_prediction: Expected 3 args");
    if (nlhs > 1) mexErrMsgTxt("covariance_prediction: Expected 1 output");
    
    Matrix15x15 P, F, Q, P_new;
    
    if (!matToMatrix<15, 15>(prhs[1], P)) mexErrMsgTxt("P must be 15x15");
    if (!matToMatrix<15, 15>(prhs[2], F)) mexErrMsgTxt("F must be 15x15");
    if (!matToMatrix<15, 15>(prhs[3], Q)) mexErrMsgTxt("Q must be 15x15");
    
    ESKFMath::covariance_prediction(P, F, Q, P_new);
    plhs[0] = matrixToMat<15, 15>(P_new);
}

// Handler: inject_error_state
// [p_new, v_new, q_new, ba_new, bg_new] = mex_eskf_math('inject_error_state', p, v, q, ba, bg, dx)
static void handle_inject_error_state(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 7) mexErrMsgTxt("inject_error_state: Expected 6 args");
    if (nlhs > 5) mexErrMsgTxt("inject_error_state: Expected 5 outputs");
    
    Vector3 p_in, v_in, ba_in, bg_in, p_out, v_out, ba_out, bg_out;
    Vector4 q_in, q_out;
    Vector15 dx;
    
    if (!matToVector<3>(prhs[1], p_in)) mexErrMsgTxt("p must be 3x1");
    if (!matToVector<3>(prhs[2], v_in)) mexErrMsgTxt("v must be 3x1");
    if (!matToVector<4>(prhs[3], q_in)) mexErrMsgTxt("q must be 4x1");
    if (!matToVector<3>(prhs[4], ba_in)) mexErrMsgTxt("ba must be 3x1");
    if (!matToVector<3>(prhs[5], bg_in)) mexErrMsgTxt("bg must be 3x1");
    if (!matToVector<15>(prhs[6], dx)) mexErrMsgTxt("dx must be 15x1");
    
    ESKFMath::inject_error_state(p_in, v_in, q_in, ba_in, bg_in, dx, 
                                  p_out, v_out, q_out, ba_out, bg_out);
    
    plhs[0] = vectorToMat<3>(p_out);
    if (nlhs > 1) plhs[1] = vectorToMat<3>(v_out);
    if (nlhs > 2) plhs[2] = vectorToMat<4>(q_out);
    if (nlhs > 3) plhs[3] = vectorToMat<3>(ba_out);
    if (nlhs > 4) plhs[4] = vectorToMat<3>(bg_out);
}

// Handler: kalman_update (15-state, 3-measurement)
// [x_new, P_new, K, S] = mex_eskf_math('kalman_update', x, P, y, H, R)
static void handle_kalman_update(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 6) mexErrMsgTxt("kalman_update: Expected 5 args");
    if (nlhs > 4) mexErrMsgTxt("kalman_update: Expected up to 4 outputs");
    
    // Determine dimensions from inputs
    int N = (int)mxGetM(prhs[1]);  // State dimension from x
    int M = (int)mxGetM(prhs[3]);  // Measurement dimension from y
    
    // Only support common cases: N=15, M=1,2,3
    if (N != 15) {
        mexErrMsgTxt("kalman_update: Only N=15 supported currently");
    }
    
    if (M == 3) {
        Vector15 x_in, x_out;
        Matrix15x15 P_in, P_out;
        Vector3 y;
        cmath_fx::Matrix<3, 15, Scalar> H;
        Matrix3x3 R;
        cmath_fx::Matrix<15, 3, Scalar> K;
        Matrix3x3 S;
        
        if (!matToVector<15>(prhs[1], x_in)) mexErrMsgTxt("x must be 15x1");
        if (!matToMatrix<15, 15>(prhs[2], P_in)) mexErrMsgTxt("P must be 15x15");
        if (!matToVector<3>(prhs[3], y)) mexErrMsgTxt("y must be 3x1");
        if (!matToMatrix<3, 15>(prhs[4], H)) mexErrMsgTxt("H must be 3x15");
        if (!matToMatrix<3, 3>(prhs[5], R)) mexErrMsgTxt("R must be 3x3");
        
        ESKFMath::kalman_update<15, 3>(x_in, P_in, y, H, R, x_out, P_out, K, S);
        
        plhs[0] = vectorToMat<15>(x_out);
        if (nlhs > 1) plhs[1] = matrixToMat<15, 15>(P_out);
        if (nlhs > 2) plhs[2] = matrixToMat<15, 3>(K);
        if (nlhs > 3) plhs[3] = matrixToMat<3, 3>(S);
        
    } else if (M == 1) {
        Vector15 x_in, x_out;
        Matrix15x15 P_in, P_out;
        cmath_fx::Vector<1, Scalar> y;
        cmath_fx::Matrix<1, 15, Scalar> H;
        cmath_fx::Matrix<1, 1, Scalar> R;
        cmath_fx::Matrix<15, 1, Scalar> K;
        cmath_fx::Matrix<1, 1, Scalar> S;
        
        if (!matToVector<15>(prhs[1], x_in)) mexErrMsgTxt("x must be 15x1");
        if (!matToMatrix<15, 15>(prhs[2], P_in)) mexErrMsgTxt("P must be 15x15");
        if (!matToVector<1>(prhs[3], y)) mexErrMsgTxt("y must be 1x1");
        if (!matToMatrix<1, 15>(prhs[4], H)) mexErrMsgTxt("H must be 1x15");
        if (!matToMatrix<1, 1>(prhs[5], R)) mexErrMsgTxt("R must be 1x1");
        
        ESKFMath::kalman_update<15, 1>(x_in, P_in, y, H, R, x_out, P_out, K, S);
        
        plhs[0] = vectorToMat<15>(x_out);
        if (nlhs > 1) plhs[1] = matrixToMat<15, 15>(P_out);
        if (nlhs > 2) plhs[2] = matrixToMat<15, 1>(K);
        if (nlhs > 3) plhs[3] = matrixToMat<1, 1>(S);
        
    } else {
        mexErrMsgTxt("kalman_update: Measurement dimension must be 1 or 3");
    }
}

// Handler: mag_observation_prediction
// m_body = mex_eskf_math('mag_observation_prediction', q, m_world)
static void handle_mag_observation(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 3) mexErrMsgTxt("mag_observation_prediction: Expected 2 args");
    if (nlhs > 1) mexErrMsgTxt("mag_observation_prediction: Expected 1 output");
    
    Vector4 q;
    Vector3 m_world, m_body;
    
    if (!matToVector<4>(prhs[1], q)) mexErrMsgTxt("q must be 4x1");
    if (!matToVector<3>(prhs[2], m_world)) mexErrMsgTxt("m_world must be 3x1");
    
    ESKFMath::mag_observation_prediction(q, m_world, m_body);
    plhs[0] = vectorToMat<3>(m_body);
}

// Handler: gps_to_local
// pos_local = mex_eskf_math('gps_to_local', gps_pos, origin_pos)
static void handle_gps_to_local(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 3) mexErrMsgTxt("gps_to_local: Expected 2 args");
    if (nlhs > 1) mexErrMsgTxt("gps_to_local: Expected 1 output");
    
    Vector3 gps_pos, origin_pos, local_pos;
    
    if (!matToVector<3>(prhs[1], gps_pos)) mexErrMsgTxt("gps_pos must be 3x1");
    if (!matToVector<3>(prhs[2], origin_pos)) mexErrMsgTxt("origin_pos must be 3x1");
    
    ESKFMath::gps_to_local(gps_pos, origin_pos, local_pos);
    plhs[0] = vectorToMat<3>(local_pos);
}

// Handler: pressure_to_altitude
// altitude = mex_eskf_math('pressure_to_altitude', pressure)
static void handle_pressure_to_altitude(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) mexErrMsgTxt("pressure_to_altitude: Expected 1 arg");
    if (nlhs > 1) mexErrMsgTxt("pressure_to_altitude: Expected 1 output");
    
    Scalar pressure = getScalar(prhs[1]);
    Scalar altitude = ESKFMath::pressure_to_altitude(pressure);
    
    plhs[0] = mxCreateDoubleScalar(static_cast<double>(altitude));
}

// Main MEX entry point
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) {
        mexErrMsgTxt("Usage: mex_eskf_math('function_name', args...)");
    }
    
    if (!mxIsChar(prhs[0])) {
        mexErrMsgTxt("First argument must be a function name string");
    }
    
    char func_name[128];
    mxGetString(prhs[0], func_name, sizeof(func_name));
    
    try {
        if (strcmp(func_name, "quaternion_integration") == 0) {
            handle_quaternion_integration(nlhs, plhs, nrhs, prhs);
        } else if (strcmp(func_name, "accel_to_quaternion") == 0) {
            handle_accel_to_quaternion(nlhs, plhs, nrhs, prhs);
        } else if (strcmp(func_name, "pv_integration") == 0) {
            handle_pv_integration(nlhs, plhs, nrhs, prhs);
        } else if (strcmp(func_name, "compute_F_matrix") == 0) {
            handle_compute_F_matrix(nlhs, plhs, nrhs, prhs);
        } else if (strcmp(func_name, "covariance_prediction") == 0) {
            handle_covariance_prediction(nlhs, plhs, nrhs, prhs);
        } else if (strcmp(func_name, "inject_error_state") == 0) {
            handle_inject_error_state(nlhs, plhs, nrhs, prhs);
        } else if (strcmp(func_name, "kalman_update") == 0) {
            handle_kalman_update(nlhs, plhs, nrhs, prhs);
        } else if (strcmp(func_name, "mag_observation_prediction") == 0) {
            handle_mag_observation(nlhs, plhs, nrhs, prhs);
        } else if (strcmp(func_name, "gps_to_local") == 0) {
            handle_gps_to_local(nlhs, plhs, nrhs, prhs);
        } else if (strcmp(func_name, "pressure_to_altitude") == 0) {
            handle_pressure_to_altitude(nlhs, plhs, nrhs, prhs);
        } else {
            mexErrMsgTxt("Unknown function name");
        }
    } catch (const std::exception& e) {
        mexErrMsgTxt(e.what());
    } catch (...) {
        mexErrMsgTxt("Unknown error occurred");
    }
}
