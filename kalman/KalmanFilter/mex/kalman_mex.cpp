/*
 * kalman_mex.cpp
 *
 * MEX wrapper around KalmanFilter C++ class (kalman_filter.hpp / kalman_filter.cpp)
 * Usage from MATLAB (after compiling MEX):
 *   h = kalman_mex('new');
 *   kalman_mex('init', h, state_size, obs_size);
 *   kalman_mex('setSystem', h, F);          % F: state x state
 *   kalman_mex('setObservationMatrix', h, H);% H: obs x state
 *   kalman_mex('setPrediction', h, pred);   % pred: state x 1
 *   kalman_mex('setObservation', h, obs);   % obs: obs x 1
 *   kalman_mex('update', h);
 *   out = kalman_mex('get', h);             % out: state x 1
 *   kalman_mex('estimateNoise', h, meas);
 *   kalman_mex('delete', h);
 *
 * Compile in MATLAB (from project root or this folder):
 *   mex('kalman_mex.cpp', '..\\..\\cpp\\src\\kalman_filter.cpp')
 *
 */

#include "mex.h"
#include <cstdint>
#include <cstring>
#include "../../cpp/inc/kalman_filter.hpp"

// Helper: check input argument count
void mexErrIfNotArguments(int nlhs, int nrhs, int need_nlhs, int need_nrhs) {
    if (nrhs < need_nrhs) mexErrMsgIdAndTxt("kalman_mex:args","Not enough input arguments");
}

// Create a MATLAB uint64 scalar to hold pointer
mxArray* createHandle(KalmanFilter* ptr) {
    mxArray* out = mxCreateNumericMatrix(1,1,mxUINT64_CLASS,mxREAL);
    uint64_t* data = (uint64_t*)mxGetData(out);
    data[0] = reinterpret_cast<uint64_t>(ptr);
    return out;
}

KalmanFilter* getHandle(const mxArray* arr) {
    if (!mxIsNumeric(const_cast<mxArray*>(arr)) || mxGetNumberOfElements(arr) != 1) {
        mexErrMsgIdAndTxt("kalman_mex:badhandle","Handle must be a numeric scalar (uint64)");
    }
    uint64_t val = 0;
    if (mxIsUint64(const_cast<mxArray*>(arr))) {
        val = *(uint64_t*)mxGetData(const_cast<mxArray*>(arr));
    } else {
        // accept double as fallback
        double d = mxGetScalar(arr);
        val = static_cast<uint64_t>(d);
    }
    return reinterpret_cast<KalmanFilter*>(val);
}

// Convert MATLAB double matrix to float buffer, column-major to row-major mapping assumed by user
// In this project, C++ expects row-major flattened as row*cols + col, but MATLAB stores column-major.
// We'll map MATLAB (m x n) to C buffer with same logical matrix: C[i*cols + j] = matlab(j,i)
void copyMatToFloat(const mxArray* mat, float* dst, mwSize expectedRows, mwSize expectedCols) {
    mwSize rows = mxGetM(mat);
    mwSize cols = mxGetN(mat);
    if (rows != expectedRows || cols != expectedCols) mexErrMsgIdAndTxt("kalman_mex:size","Input matrix has incorrect size.");
    if (mxIsSingle(const_cast<mxArray*>(mat))) {
        float* src = (float*)mxGetData(mat);
        for (mwSize r = 0; r < rows; ++r) {
            for (mwSize c = 0; c < cols; ++c) {
                dst[r * cols + c] = src[c * rows + r];
            }
        }
    } else if (mxIsDouble(const_cast<mxArray*>(mat))) {
        double* src = mxGetPr(mat);
        for (mwSize r = 0; r < rows; ++r) {
            for (mwSize c = 0; c < cols; ++c) {
                dst[r * cols + c] = static_cast<float>(src[c * rows + r]);
            }
        }
    } else {
        mexErrMsgIdAndTxt("kalman_mex:type","Input must be single or double.");
    }
}

void copyVecToFloat(const mxArray* vec, float* dst, mwSize expectedLen) {
    mwSize rows = mxGetM(vec);
    mwSize cols = mxGetN(vec);
    mwSize len = rows * cols;
    if (len != expectedLen) mexErrMsgIdAndTxt("kalman_mex:size","Input vector has incorrect length.");
    if (mxIsSingle(const_cast<mxArray*>(vec))) {
        float* src = (float*)mxGetData(vec);
        for (mwSize i = 0; i < len; ++i) dst[i] = src[i];
    } else if (mxIsDouble(const_cast<mxArray*>(vec))) {
        double* src = mxGetPr(vec);
        for (mwSize i = 0; i < len; ++i) dst[i] = static_cast<float>(src[i]);
    } else {
        mexErrMsgIdAndTxt("kalman_mex:type","Input must be single or double.");
    }
}

mxArray* createDoubleVecFromFloat(const float* src, mwSize len) {
    mxArray* out = mxCreateDoubleMatrix(len,1,mxREAL);
    double* dst = mxGetPr(out);
    for (mwSize i = 0; i < len; ++i) dst[i] = static_cast<double>(src[i]);
    return out;
}

mxArray* createSingleVecFromFloat(const float* src, mwSize len) {
    mxArray* out = mxCreateNumericMatrix(len,1,mxSINGLE_CLASS,mxREAL);
    float* dst = (float*)mxGetData(out);
    for (mwSize i = 0; i < len; ++i) dst[i] = src[i];
    return out;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    if (nrhs < 1) mexErrMsgIdAndTxt("kalman_mex:cmd","First argument must be a command string.");
    if (!mxIsChar(prhs[0])) mexErrMsgIdAndTxt("kalman_mex:cmdtype","First argument must be a command string.");
    char cmd[64];
    mxGetString(prhs[0], cmd, sizeof(cmd));

    if (strcmp(cmd, "new") == 0) {
        KalmanFilter* kf = new KalmanFilter();
        mexLock();
        plhs[0] = createHandle(kf);
        return;
    }

    // commands that require a handle as second arg
    if (nrhs < 2) mexErrMsgIdAndTxt("kalman_mex:args","Handle argument required.");
    KalmanFilter* kf = getHandle(prhs[1]);
    if (!kf) mexErrMsgIdAndTxt("kalman_mex:badhandle","Null handle.");

    if (strcmp(cmd, "delete") == 0) {
        delete kf;
        mexUnlock();
        return;
    }

    if (strcmp(cmd, "init") == 0) {
        if (nrhs < 4) mexErrMsgIdAndTxt("kalman_mex:args","init requires handle, state_size, obs_size");
        int state = static_cast<int>(mxGetScalar(prhs[2]));
        int obs = static_cast<int>(mxGetScalar(prhs[3]));
        kf->Init(static_cast<uint8_t>(state), static_cast<uint8_t>(obs));
        return;
    }

    if (strcmp(cmd, "setSystem") == 0) {
        if (nrhs < 3) mexErrMsgIdAndTxt("kalman_mex:args","setSystem requires handle and F matrix");
        // determine sizes from header/state: user must pass correct sized matrix
        // We'll try to infer state size from dimensions
        mwSize rows = mxGetM(prhs[2]);
        mwSize cols = mxGetN(prhs[2]);
        if (rows != cols) mexErrMsgIdAndTxt("kalman_mex:size","System matrix must be square.");
        copyMatToFloat(prhs[2], kf->system_matrix, rows, cols);
        return;
    }

    if (strcmp(cmd, "setQMatrix") == 0) {
        if (nrhs < 3) mexErrMsgIdAndTxt("kalman_mex:args","setQMatrix requires handle and Q matrix");
        mwSize rows = mxGetM(prhs[2]);
        mwSize cols = mxGetN(prhs[2]);
        if (rows != cols) mexErrMsgIdAndTxt("kalman_mex:size","Q must be square");
        // temporary buffer
        float Qbuf[STATE_SIZE_MAX * STATE_SIZE_MAX];
        copyMatToFloat(prhs[2], Qbuf, rows, cols);
        kf->SetQMatrix(Qbuf);
        return;
    }

    if (strcmp(cmd, "setRMatrix") == 0) {
        if (nrhs < 3) mexErrMsgIdAndTxt("kalman_mex:args","setRMatrix requires handle and R matrix");
        mwSize rows = mxGetM(prhs[2]);
        mwSize cols = mxGetN(prhs[2]);
        if (rows != cols) mexErrMsgIdAndTxt("kalman_mex:size","R must be square");
        float Rbuf[OBS_SIZE_MAX * OBS_SIZE_MAX];
        copyMatToFloat(prhs[2], Rbuf, rows, cols);
        kf->SetRMatrix(Rbuf);
        return;
    }

    if (strcmp(cmd, "setObservationMatrix") == 0) {
        if (nrhs < 3) mexErrMsgIdAndTxt("kalman_mex:args","setObservationMatrix requires handle and H matrix");
        mwSize rows = mxGetM(prhs[2]);
        mwSize cols = mxGetN(prhs[2]);
        copyMatToFloat(prhs[2], kf->observation_matrix, rows, cols);
        return;
    }

    if (strcmp(cmd, "setPrediction") == 0) {
        if (nrhs < 3) mexErrMsgIdAndTxt("kalman_mex:args","setPrediction requires handle and prediction vector");
        // vector length may be (state x 1) or (1 x state)
        mwSize rows = mxGetM(prhs[2]);
        mwSize cols = mxGetN(prhs[2]);
        mwSize len = rows * cols;
        copyVecToFloat(prhs[2], kf->prediction, len);
        return;
    }

    if (strcmp(cmd, "setObservation") == 0) {
        if (nrhs < 3) mexErrMsgIdAndTxt("kalman_mex:args","setObservation requires handle and observation vector");
        mwSize rows = mxGetM(prhs[2]);
        mwSize cols = mxGetN(prhs[2]);
        mwSize len = rows * cols;
        copyVecToFloat(prhs[2], kf->observation, len);
        return;
    }

    if (strcmp(cmd, "update") == 0) {
        kf->Update();
        return;
    }

    if (strcmp(cmd, "get") == 0) {
        float tmp[STATE_SIZE_MAX];
        kf->GetData(tmp);
        mwSize len = STATE_SIZE_MAX;
        while (len>1 && tmp[len-1]==0.0f) len--;
        plhs[0] = createSingleVecFromFloat(tmp, len);
        return;
    }

    if (strcmp(cmd, "estimateNoise") == 0) {
        if (nrhs < 3) mexErrMsgIdAndTxt("kalman_mex:args","estimateNoise requires handle and measurement scalar");
        double meas = mxGetScalar(prhs[2]);
        kf->EstimateNoise(static_cast<float>(meas));
        return;
    }

    mexErrMsgIdAndTxt("kalman_mex:unknown","Unknown command '%s'", cmd);
}