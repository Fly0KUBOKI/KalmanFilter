#include "mex.h"
#include "../Common/Math/quaternion_lib.hpp"
#include <cstring>
#include "mex_type_conv.hpp"
#include <vector>

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
        
        float q_tmp[4];
        mex_conv::mxArrayToFloatArray(prhs[1], q_tmp, 4);
        Quat q(q_tmp[0], q_tmp[1], q_tmp[2], q_tmp[3]);
        q.normalize();
        
        float R[9];
        q.to_rotation_matrix(R);
        
        plhs[0] = mxCreateDoubleMatrix(3, 3, mxREAL);
        double* R_out = mxGetPr(plhs[0]);
        // C++はrow-major、MATLABはcolumn-major → 転置が必要
        R_out[0] = R[0]; R_out[3] = R[1]; R_out[6] = R[2];
        R_out[1] = R[3]; R_out[4] = R[4]; R_out[7] = R[5];
        R_out[2] = R[6]; R_out[5] = R[7]; R_out[8] = R[8];
        
    } else if (strcmp(action, "to_euler") == 0) {
        // [roll, pitch, yaw] = to_euler(q)
        if (nrhs < 2) {
            mexErrMsgIdAndTxt("mex_quaternion_lib:nrhs", "q required");
        }
        
        float q_tmp[4];
        mex_conv::mxArrayToFloatArray(prhs[1], q_tmp, 4);
        Quat q(q_tmp[0], q_tmp[1], q_tmp[2], q_tmp[3]);
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
            float euler_tmp[3];
            mex_conv::mxArrayToFloatArray(prhs[1], euler_tmp, 3);
            roll = euler_tmp[0];
            pitch = euler_tmp[1];
            yaw = euler_tmp[2];
        } else if (nrhs == 4) {
            roll = mex_conv::mxGetScalarAsFloat(prhs[1]);
            pitch = mex_conv::mxGetScalarAsFloat(prhs[2]);
            yaw = mex_conv::mxGetScalarAsFloat(prhs[3]);
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
        
        float q_tmp2[4];
        mex_conv::mxArrayToFloatArray(prhs[1], q_tmp2, 4);
        Quat q(q_tmp2[0], q_tmp2[1], q_tmp2[2], q_tmp2[3]);
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
        
        float q1_tmp[4], q2_tmp[4];
        mex_conv::mxArrayToFloatArray(prhs[1], q1_tmp, 4);
        mex_conv::mxArrayToFloatArray(prhs[2], q2_tmp, 4);

        Quat q1(q1_tmp[0], q1_tmp[1], q1_tmp[2], q1_tmp[3]);
        Quat q2(q2_tmp[0], q2_tmp[1], q2_tmp[2], q2_tmp[3]);
        
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
        
        float theta_tmp[3];
        mex_conv::mxArrayToFloatArray(prhs[1], theta_tmp, 3);
        Quat q = Quat::from_small_angle(theta_tmp[0], theta_tmp[1], theta_tmp[2]);
        
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
        
        float q_tmp3[4]; float omega_tmp[3];
        mex_conv::mxArrayToFloatArray(prhs[1], q_tmp3, 4);
        mex_conv::mxArrayToFloatArray(prhs[2], omega_tmp, 3);
        float dt = mex_conv::mxGetScalarAsFloat(prhs[3]);

        Quat q(q_tmp3[0], q_tmp3[1], q_tmp3[2], q_tmp3[3]);
        Quat q_new = Quat::integrate(q, omega_tmp[0], omega_tmp[1], omega_tmp[2], dt);
        
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
        
        float q_tmp4[4];
        mex_conv::mxArrayToFloatArray(prhs[1], q_tmp4, 4);
        Quat q(q_tmp4[0], q_tmp4[1], q_tmp4[2], q_tmp4[3]);
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
        
        float q_tmp5[4];
        mex_conv::mxArrayToFloatArray(prhs[1], q_tmp5, 4);
        Quat q(q_tmp5[0], q_tmp5[1], q_tmp5[2], q_tmp5[3]);
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
        
        float v1_tmp[3], v2_tmp[3];
        mex_conv::mxArrayToFloatArray(prhs[1], v1_tmp, 3);
        mex_conv::mxArrayToFloatArray(prhs[2], v2_tmp, 3);

        Quat q = Quat::from_two_vectors(v1_tmp[0], v1_tmp[1], v1_tmp[2], v2_tmp[0], v2_tmp[1], v2_tmp[2]);
        
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
        
        float q1_tmp2[4], q2_tmp2[4];
        mex_conv::mxArrayToFloatArray(prhs[1], q1_tmp2, 4);
        mex_conv::mxArrayToFloatArray(prhs[2], q2_tmp2, 4);

        Quat q1(q1_tmp2[0], q1_tmp2[1], q1_tmp2[2], q1_tmp2[3]);
        Quat q2(q2_tmp2[0], q2_tmp2[1], q2_tmp2[2], q2_tmp2[3]);

        float d = Quat::distance(q1, q2);
        
        plhs[0] = mxCreateDoubleScalar(static_cast<double>(d));
        
    } else if (strcmp(action, "slerp") == 0) {
        // q = slerp(q1, q2, t)
        if (nrhs < 4) {
            mexErrMsgIdAndTxt("mex_quaternion_lib:nrhs", "q1, q2, t required");
        }
        
        float q1_tmp3[4], q2_tmp3[4];
        mex_conv::mxArrayToFloatArray(prhs[1], q1_tmp3, 4);
        mex_conv::mxArrayToFloatArray(prhs[2], q2_tmp3, 4);
        float t = mex_conv::mxGetScalarAsFloat(prhs[3]);

        Quat q1(q1_tmp3[0], q1_tmp3[1], q1_tmp3[2], q1_tmp3[3]);
        Quat q2(q2_tmp3[0], q2_tmp3[1], q2_tmp3[2], q2_tmp3[3]);
        
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
        
        float v_tmp[3];
        mex_conv::mxArrayToFloatArray(prhs[1], v_tmp, 3);
        float S[9];
        Quat::skew_symmetric(v_tmp[0], v_tmp[1], v_tmp[2], S);
        
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
