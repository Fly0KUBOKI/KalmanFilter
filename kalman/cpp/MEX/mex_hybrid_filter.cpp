/* mex_hybrid_filter.cpp
 * Hybrid Filter MEX Interface (ESKF Prediction + MEUKF Update)
 *
 * ARCHITECTURE:
 *   - Prediction Step: ESKF (Error-State Kalman Filter)
 *     └─ HybridFilterRunner::predict() → HybridFilterCore::integrate_nominal() + predict_covariance()
 *   - Update Step: MEUKF (Multiplicative Extended UKF)
 *     └─ do_sensor_update() → MEUKFCore::step() → UKF-based sensor updates
 *
 * NOTE: Despite the name "eskf", this implementation uses MEUKF for all
 *       sensor updates (Accel, Mag, GPS, Baro). See IMPLEMENTATION_ANALYSIS.md.
 *
 * API:
 *   handle = mex_hybrid_filter('init', obs, static_time, dt)
 *   mex_hybrid_filter('step', handle, obs, k)
 *   state = mex_hybrid_filter('get_state', handle)
 *   mex_hybrid_filter('free', handle)
 */

// 共通インクルードと定義（ヘッダーに移動済み）
#include "Impl/mex_hybrid_filter_common.hpp"
#include "Impl/mex_hybrid_filter_impl.hpp"

using namespace mex_hybrid_filter_impl;

// グローバル変数の実装（名前空間内で定義）
namespace mex_hybrid_filter_impl {
    struct StateEntry { uint64_t handle; FilterState* state; bool used; };
    static const int MAX_STATES = 100;
    static StateEntry g_state_table[MAX_STATES] = {};
    static uint64_t g_next_handle = 1;
    SensorFilterLib g_filter_lib;  // Global sensor filter library instance

    uint64_t allocate_handle(FilterState* s) {
        for (int i = 0; i < MAX_STATES; ++i) {
            if (!g_state_table[i].used) {
                g_state_table[i].used = true;
                g_state_table[i].handle = g_next_handle++;
                g_state_table[i].state = s;
                return g_state_table[i].handle;
            }
        }
        mexErrMsgIdAndTxt("mex_hybrid_filter:alloc", "Max state handles reached (%d)", MAX_STATES);
        return 0;
    }

    FilterState* find_state(uint64_t handle) {
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
    if (nrhs < 1) mexErrMsgIdAndTxt("mex_hybrid_filter:usage", "Command required");
    
    // Extract command string (direct implementation without std::string)
    char cmd_buf[64] = {0};
    if (!mxIsChar(prhs[0])) mexErrMsgIdAndTxt("mex_hybrid_filter:usage", "Command must be a string");
    if (mxGetString(prhs[0], cmd_buf, sizeof(cmd_buf)) != 0) mexErrMsgIdAndTxt("mex_hybrid_filter:usage", "Command string too long");
    
    if (std::strcmp(cmd_buf, "init") == 0) {
        if (nrhs < 4) mexErrMsgIdAndTxt("mex_hybrid_filter:usage", "init requires (obs, static_time, dt)");
        const mxArray* obs = prhs[1];
        double static_time = mxGetScalar(prhs[2]);
        double dt = mxGetScalar(prhs[3]);
        uint64_t handle = do_init(obs, static_time, dt);
        plhs[0] = mxCreateNumericMatrix(1, 1, mxUINT64_CLASS, mxREAL);
        *((uint64_t*)mxGetData(plhs[0])) = handle;
    }
    else if (std::strcmp(cmd_buf, "step") == 0) {
        if (nrhs < 4) mexErrMsgIdAndTxt("mex_hybrid_filter:usage", "step requires (handle, obs, k)");
        uint64_t handle = *((uint64_t*)mxGetData(prhs[1]));
        const mxArray* obs = prhs[2];
        int k = (int)mxGetScalar(prhs[3]);
        FilterState* s = find_state(handle);
        if (!s) mexErrMsgIdAndTxt("mex_hybrid_filter:invalid", "Invalid handle");
        do_step(s, obs, k);
    }
    else if (std::strcmp(cmd_buf, "get_state") == 0) {
        if (nrhs < 2) mexErrMsgIdAndTxt("mex_hybrid_filter:usage", "get_state requires (handle)");
        uint64_t handle = *((uint64_t*)mxGetData(prhs[1]));
        FilterState* s = find_state(handle);
        if (!s) mexErrMsgIdAndTxt("mex_hybrid_filter:invalid", "Invalid handle");
        plhs[0] = do_get_state(s);
    }
    else if (std::strcmp(cmd_buf, "free") == 0) {
        if (nrhs < 2) mexErrMsgIdAndTxt("mex_hybrid_filter:usage", "free requires (handle)");
        uint64_t handle = *((uint64_t*)mxGetData(prhs[1]));
        do_free(handle);
    }
    else if (std::strcmp(cmd_buf, "meukf_step") == 0) {
        // API: mex_hybrid_filter('meukf_step', prev_state_struct, sensor_struct, params_struct)
        // 戻り値: [new_state, dbg_info, dbg_output] (mex_meukf_step_v2と互換)
        if (nrhs < 4) mexErrMsgIdAndTxt("mex_hybrid_filter:usage", "meukf_step requires (prev_state, sensor, params)");
        const mxArray* prev_state = prhs[1];
        const mxArray* sensor = prhs[2];
        const mxArray* params = prhs[3];
        mxArray* new_state = nullptr;
        mxArray* dbg_out = nullptr;
        mxArray* dbg_output = nullptr;
        do_sensor_update(prev_state, sensor, params, new_state, dbg_out, dbg_output);
        if (nlhs > 0) plhs[0] = new_state;
        if (nlhs > 1) plhs[1] = dbg_out;
        if (nlhs > 2) plhs[2] = dbg_output;
    }
    else if (std::strcmp(cmd_buf, "sensor_filter_reset_zero") == 0) {
        plhs[0] = do_sensor_filter_reset_zero();
    }
    else if (std::strcmp(cmd_buf, "sensor_filter_reset") == 0) {
        plhs[0] = do_sensor_filter_reset();
    }
    else if (std::strcmp(cmd_buf, "sensor_filter_update") == 0) {
        if (nrhs < 2) mexErrMsgIdAndTxt("mex_hybrid_filter:usage", "sensor_filter_update requires (sensor_struct)");
        const mxArray* sensor = prhs[1];
        plhs[0] = do_sensor_filter_update(sensor);
    }
    else {
        mexErrMsgIdAndTxt("mex_hybrid_filter:unknown", "Unknown command: %s", cmd_buf);
    }
}
