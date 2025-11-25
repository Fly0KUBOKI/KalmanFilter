#include "mex.h"
#include "../Common/Math/quaternion_lib.hpp"
#include <cstring>

using Quat = quat_lib::Quaternion<float>;

// MEX関数: mex_quaternion_lib
// 使用法:
//   R = mex_quaternion_lib('to_rotation_matrix', q)
//   [roll, pitch, yaw] = mex_quaternion_lib('to_euler', q)
//   q = mex_quaternion_lib('from_euler', roll, pitch, yaw)
//   q = mex_quaternion_lib('normalize', q)
//   q = mex_quaternion_lib('multiply', q1, q2)
//   q = mex_quaternion_lib('small_angle_quat', theta)
//   q = mex_quaternion_lib('integrate', q, omega, dt)
//   等

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if (nrhs < 1) {
        mexErrMsgIdAndTxt("mex_quaternion_lib:nrhs",
                          "At least 1 input required: action");
    }
    
    // アクション文字列取得
    char action[128];
    if (mxGetString(prhs[0], action, sizeof(action)) != 0) {
        mexErrMsgIdAndTxt("mex_quaternion_lib:action",
                          "First argument must be a string");
    }
    
    // アクション別処理
    if (strcmp(action, "to_rotation_matrix") == 0) {
        // R = to_rotation_matrix(q)
        if (nrhs < 2) {
            mexErrMsgIdAndTxt("mex_quaternion_lib:nrhs", "q required");
        }
        
        const double* q_data = mxGetPr(prhs[1]);
        Quat q(q_data[0], q_data[1], q_data[2], q_data[3]);
        q.normalize();
        
        float R[9];
        q.to_rotation_matrix(R);
        
        plhs[0] = mxCreateDoubleMatrix(3, 3, mxREAL);
        double* R_out = mxGetPr(plhs[0]);
        for (int i = 0; i < 9; ++i) {
            R_out[i] = static_cast<double>(R[i]);
        }
        
    } else if (strcmp(action, "to_euler") == 0) {
        // [roll, pitch, yaw] = to_euler(q)
        if (nrhs < 2) {
            mexErrMsgIdAndTxt("mex_quaternion_lib:nrhs", "q required");
        }
        
        const double* q_data = mxGetPr(prhs[1]);
        Quat q(q_data[0], q_data[1], q_data[2], q_data[3]);
        q.normalize();
        
        float roll, pitch, yaw;
        q.to_euler(roll, pitch, yaw);
        
        plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL);
        double* euler_out = mxGetPr(plhs[0]);
        euler_out[0] = static_cast<double>(roll);
        euler_out[1] = static_cast<double>(pitch);
        euler_out[2] = static_cast<double>(yaw);
        
    } else if (strcmp(action, "from_euler") == 0) {
        // q = from_euler(euler) or q = from_euler(roll, pitch, yaw)
        float roll, pitch, yaw;
        
        if (nrhs == 2) {
            const double* euler = mxGetPr(prhs[1]);
            roll = euler[0];
            pitch = euler[1];
            yaw = euler[2];
        } else if (nrhs == 4) {
            roll = mxGetScalar(prhs[1]);
            pitch = mxGetScalar(prhs[2]);
            yaw = mxGetScalar(prhs[3]);
        } else {
            mexErrMsgIdAndTxt("mex_quaternion_lib:nrhs", "euler vector or (roll,pitch,yaw) required");
        }
        
        Quat q = Quat::from_euler(roll, pitch, yaw);
        
        plhs[0] = mxCreateDoubleMatrix(4, 1, mxREAL);
        double* q_out = mxGetPr(plhs[0]);
        q_out[0] = q.w;
        q_out[1] = q.x;
        q_out[2] = q.y;
        q_out[3] = q.z;
        
    } else if (strcmp(action, "normalize") == 0) {
        // q = normalize(q)
        if (nrhs < 2) {
            mexErrMsgIdAndTxt("mex_quaternion_lib:nrhs", "q required");
        }
        
        const double* q_data = mxGetPr(prhs[1]);
        Quat q(q_data[0], q_data[1], q_data[2], q_data[3]);
        q.normalize();
        
        plhs[0] = mxCreateDoubleMatrix(4, 1, mxREAL);
        double* q_out = mxGetPr(plhs[0]);
        q_out[0] = q.w;
        q_out[1] = q.x;
        q_out[2] = q.y;
        q_out[3] = q.z;
        
    } else if (strcmp(action, "multiply") == 0) {
        // q = multiply(q1, q2)
        if (nrhs < 3) {
            mexErrMsgIdAndTxt("mex_quaternion_lib:nrhs", "q1 and q2 required");
        }
        
        const double* q1_data = mxGetPr(prhs[1]);
        const double* q2_data = mxGetPr(prhs[2]);
        
        Quat q1(q1_data[0], q1_data[1], q1_data[2], q1_data[3]);
        Quat q2(q2_data[0], q2_data[1], q2_data[2], q2_data[3]);
        
        Quat q = Quat::multiply(q1, q2);
        
        plhs[0] = mxCreateDoubleMatrix(4, 1, mxREAL);
        double* q_out = mxGetPr(plhs[0]);
        q_out[0] = q.w;
        q_out[1] = q.x;
        q_out[2] = q.y;
        q_out[3] = q.z;
        
    } else if (strcmp(action, "small_angle_quat") == 0) {
        // q = small_angle_quat(theta)
        if (nrhs < 2) {
            mexErrMsgIdAndTxt("mex_quaternion_lib:nrhs", "theta required");
        }
        
        const double* theta = mxGetPr(prhs[1]);
        Quat q = Quat::from_small_angle(theta[0], theta[1], theta[2]);
        
        plhs[0] = mxCreateDoubleMatrix(4, 1, mxREAL);
        double* q_out = mxGetPr(plhs[0]);
        q_out[0] = q.w;
        q_out[1] = q.x;
        q_out[2] = q.y;
        q_out[3] = q.z;
        
    } else if (strcmp(action, "integrate") == 0) {
        // q_new = integrate(q, omega, dt)
        if (nrhs < 4) {
            mexErrMsgIdAndTxt("mex_quaternion_lib:nrhs", "q, omega, dt required");
        }
        
        const double* q_data = mxGetPr(prhs[1]);
        const double* omega = mxGetPr(prhs[2]);
        float dt = mxGetScalar(prhs[3]);
        
        Quat q(q_data[0], q_data[1], q_data[2], q_data[3]);
        Quat q_new = Quat::integrate(q, omega[0], omega[1], omega[2], dt);
        
        plhs[0] = mxCreateDoubleMatrix(4, 1, mxREAL);
        double* q_out = mxGetPr(plhs[0]);
        q_out[0] = q_new.w;
        q_out[1] = q_new.x;
        q_out[2] = q_new.y;
        q_out[3] = q_new.z;
        
    } else if (strcmp(action, "conjugate") == 0) {
        // q_conj = conjugate(q)
        if (nrhs < 2) {
            mexErrMsgIdAndTxt("mex_quaternion_lib:nrhs", "q required");
        }
        
        const double* q_data = mxGetPr(prhs[1]);
        Quat q(q_data[0], q_data[1], q_data[2], q_data[3]);
        Quat q_conj = q.conjugate();
        
        plhs[0] = mxCreateDoubleMatrix(4, 1, mxREAL);
        double* q_out = mxGetPr(plhs[0]);
        q_out[0] = q_conj.w;
        q_out[1] = q_conj.x;
        q_out[2] = q_conj.y;
        q_out[3] = q_conj.z;
        
    } else if (strcmp(action, "inverse") == 0) {
        // q_inv = inverse(q)
        if (nrhs < 2) {
            mexErrMsgIdAndTxt("mex_quaternion_lib:nrhs", "q required");
        }
        
        const double* q_data = mxGetPr(prhs[1]);
        Quat q(q_data[0], q_data[1], q_data[2], q_data[3]);
        Quat q_inv = q.inverse();
        
        plhs[0] = mxCreateDoubleMatrix(4, 1, mxREAL);
        double* q_out = mxGetPr(plhs[0]);
        q_out[0] = q_inv.w;
        q_out[1] = q_inv.x;
        q_out[2] = q_inv.y;
        q_out[3] = q_inv.z;
        
    } else if (strcmp(action, "from_two_vectors") == 0) {
        // q = from_two_vectors(v1, v2)
        if (nrhs < 3) {
            mexErrMsgIdAndTxt("mex_quaternion_lib:nrhs", "v1 and v2 required");
        }
        
        const double* v1 = mxGetPr(prhs[1]);
        const double* v2 = mxGetPr(prhs[2]);
        
        Quat q = Quat::from_two_vectors(v1[0], v1[1], v1[2], v2[0], v2[1], v2[2]);
        
        plhs[0] = mxCreateDoubleMatrix(4, 1, mxREAL);
        double* q_out = mxGetPr(plhs[0]);
        q_out[0] = q.w;
        q_out[1] = q.x;
        q_out[2] = q.y;
        q_out[3] = q.z;
        
    } else if (strcmp(action, "distance") == 0) {
        // d = distance(q1, q2)
        if (nrhs < 3) {
            mexErrMsgIdAndTxt("mex_quaternion_lib:nrhs", "q1 and q2 required");
        }
        
        const double* q1_data = mxGetPr(prhs[1]);
        const double* q2_data = mxGetPr(prhs[2]);
        
        Quat q1(q1_data[0], q1_data[1], q1_data[2], q1_data[3]);
        Quat q2(q2_data[0], q2_data[1], q2_data[2], q2_data[3]);
        
        float d = Quat::distance(q1, q2);
        
        plhs[0] = mxCreateDoubleScalar(static_cast<double>(d));
        
    } else if (strcmp(action, "slerp") == 0) {
        // q = slerp(q1, q2, t)
        if (nrhs < 4) {
            mexErrMsgIdAndTxt("mex_quaternion_lib:nrhs", "q1, q2, t required");
        }
        
        const double* q1_data = mxGetPr(prhs[1]);
        const double* q2_data = mxGetPr(prhs[2]);
        float t = mxGetScalar(prhs[3]);
        
        Quat q1(q1_data[0], q1_data[1], q1_data[2], q1_data[3]);
        Quat q2(q2_data[0], q2_data[1], q2_data[2], q2_data[3]);
        
        Quat q = Quat::slerp(q1, q2, t);
        
        plhs[0] = mxCreateDoubleMatrix(4, 1, mxREAL);
        double* q_out = mxGetPr(plhs[0]);
        q_out[0] = q.w;
        q_out[1] = q.x;
        q_out[2] = q.y;
        q_out[3] = q.z;
        
    } else if (strcmp(action, "skew") == 0) {
        // S = skew(v)
        if (nrhs < 2) {
            mexErrMsgIdAndTxt("mex_quaternion_lib:nrhs", "v required");
        }
        
        const double* v = mxGetPr(prhs[1]);
        float S[9];
        Quat::skew_symmetric(v[0], v[1], v[2], S);
        
        plhs[0] = mxCreateDoubleMatrix(3, 3, mxREAL);
        double* S_out = mxGetPr(plhs[0]);
        for (int i = 0; i < 9; ++i) {
            S_out[i] = static_cast<double>(S[i]);
        }
        
    } else {
        mexErrMsgIdAndTxt("mex_quaternion_lib:action",
                          "Unknown action: %s", action);
    }
}
