/* mex_run_eskf.cpp
 * Hybrid Filter MEX Interface (ESKF Prediction + MEUKF Update)
 *
 * ARCHITECTURE:
 *   - Prediction Step: ESKF (Error-State Kalman Filter)
 *     └─ ESKFRunner::predict() → ESKFCore::integrate_nominal() + predict_covariance()
 *   - Update Step: MEUKF (Multiplicative Extended UKF)
 *     └─ do_sensor_update_meukf() → MEUKFCore::step() → UKF-based sensor updates
 *
 * NOTE: Despite the name "eskf", this implementation uses MEUKF for all
 *       sensor updates (Accel, Mag, GPS, Baro). See IMPLEMENTATION_ANALYSIS.md.
 *
 * API:
 *   handle = mex_run_eskf('init', obs, static_time, dt)
 *   mex_run_eskf('step', handle, obs, k)
 *   state = mex_run_eskf('get_state', handle)
 *   mex_run_eskf('free', handle)
 */

// 共通インクルードと定義（ヘッダーに移動済み）
#include "Impl/mex_eskf_common.hpp"
#include "Impl/mex_run_eskf_impl.hpp"

using namespace mex_run_eskf_impl;

// グローバル変数の実装（名前空間内で定義）
namespace mex_run_eskf_impl {
    struct StateEntry { uint64_t handle; ESKFState* state; bool used; };
    static const int MAX_STATES = 100;
    static StateEntry g_state_table[MAX_STATES] = {};
    static uint64_t g_next_handle = 1;
    SensorFilterLib g_filter_lib;  // Global sensor filter library instance

    uint64_t allocate_handle(ESKFState* s) {
        for (int i = 0; i < MAX_STATES; ++i) {
            if (!g_state_table[i].used) {
                g_state_table[i].used = true;
                g_state_table[i].handle = g_next_handle++;
                g_state_table[i].state = s;
                return g_state_table[i].handle;
            }
        }
        mexErrMsgIdAndTxt("mex_run_eskf:alloc", "Max state handles reached (%d)", MAX_STATES);
        return 0;
    }

    ESKFState* find_state(uint64_t handle) {
        for (int i = 0; i < MAX_STATES; ++i) {
            if (g_state_table[i].used && g_state_table[i].handle == handle) return g_state_table[i].state;
        }
        return nullptr;
    }

    void remove_handle(uint64_t handle) {
        for (int i = 0; i < MAX_STATES; ++i) {
            if (g_state_table[i].used && g_state_table[i].handle == handle) {
                g_state_table[i].used = false;
                g_state_table[i].state = nullptr;
                g_state_table[i].handle = 0;
                return;
            }
        }
    }
}

// mexFunctionのみを実装（他の関数はすべてヘッダーに移動済み）
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) mexErrMsgIdAndTxt("mex_run_eskf:usage", "Command required");
    std::string cmd = getCmd(prhs[0]);
    
    if (cmd == "init") {
        if (nrhs < 4) mexErrMsgIdAndTxt("mex_run_eskf:usage", "init requires (obs, static_time, dt)");
        const mxArray* obs = prhs[1];
        double static_time = mxGetScalar(prhs[2]);
        double dt = mxGetScalar(prhs[3]);
        uint64_t handle = do_init(obs, static_time, dt);
        plhs[0] = mxCreateNumericMatrix(1, 1, mxUINT64_CLASS, mxREAL);
        *((uint64_t*)mxGetData(plhs[0])) = handle;
    }
    else if (cmd == "step") {
        if (nrhs < 4) mexErrMsgIdAndTxt("mex_run_eskf:usage", "step requires (handle, obs, k)");
        uint64_t handle = *((uint64_t*)mxGetData(prhs[1]));
        const mxArray* obs = prhs[2];
        int k = (int)mxGetScalar(prhs[3]);
        ESKFState* s = find_state(handle);
        if (!s) mexErrMsgIdAndTxt("mex_run_eskf:invalid", "Invalid handle");
        do_step(s, obs, k);
    }
    else if (cmd == "get_state") {
        if (nrhs < 2) mexErrMsgIdAndTxt("mex_run_eskf:usage", "get_state requires (handle)");
        uint64_t handle = *((uint64_t*)mxGetData(prhs[1]));
        ESKFState* s = find_state(handle);
        if (!s) mexErrMsgIdAndTxt("mex_run_eskf:invalid", "Invalid handle");
        plhs[0] = do_get_state(s);
    }
    else if (cmd == "free") {
        if (nrhs < 2) mexErrMsgIdAndTxt("mex_run_eskf:usage", "free requires (handle)");
        uint64_t handle = *((uint64_t*)mxGetData(prhs[1]));
        do_free(handle);
    }
    else if (cmd == "meukf_step") {
        // API: mex_run_eskf('meukf_step', prev_state_struct, sensor_struct, params_struct)
        // 戻り値: [new_state, dbg_info, dbg_output] (mex_meukf_step_v2と互換)
        if (nrhs < 4) mexErrMsgIdAndTxt("mex_run_eskf:usage", "meukf_step requires (prev_state, sensor, params)");
        const mxArray* prev_state = prhs[1];
        const mxArray* sensor = prhs[2];
        const mxArray* params = prhs[3];
        mxArray* new_state = nullptr;
        mxArray* dbg_out = nullptr;
        mxArray* dbg_output = nullptr;
        do_sensor_update_meukf(prev_state, sensor, params, new_state, dbg_out, dbg_output);
        if (nlhs > 0) plhs[0] = new_state;
        if (nlhs > 1) plhs[1] = dbg_out;
        if (nlhs > 2) plhs[2] = dbg_output;
    }
    else if (cmd == "sensor_filter_reset_zero") {
        plhs[0] = do_sensor_filter_reset_zero();
    }
    else if (cmd == "sensor_filter_reset") {
        plhs[0] = do_sensor_filter_reset();
    }
    else if (cmd == "sensor_filter_update") {
        if (nrhs < 2) mexErrMsgIdAndTxt("mex_run_eskf:usage", "sensor_filter_update requires (sensor_struct)");
        const mxArray* sensor = prhs[1];
        plhs[0] = do_sensor_filter_update(sensor);
    }
    else {
        mexErrMsgIdAndTxt("mex_run_eskf:unknown", "Unknown command: %s", cmd.c_str());
    }
}
