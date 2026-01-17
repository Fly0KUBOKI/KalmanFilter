#include "../inc/standalone.hpp"
#include "../../../ESKF/inc/filter.hpp"
#include <cstring>

namespace kalman {

// ============================================================================
// Internal wrapper to hold Filter implementation + params (FilterInstance)
// ============================================================================
struct FilterInstance {
    Filter* impl;
    Params params;
    bool initialized;
    FilterInstance(): impl(nullptr), initialized(false) {
        // default params
        params.g[0] = 0.0f; params.g[1] = 0.0f; params.g[2] = -9.81f;
        params.dt = 0.0f;
        params.mag_ref[0] = params.mag_ref[1] = params.mag_ref[2] = 0.0f;
        for (int i = 0; i < 3; i++) {
            params.noise_accel[i] = 0.0f;
            params.noise_gyro[i] = 0.0f;
            params.noise_ba[i] = 0.0f;
            params.noise_bg[i] = 0.0f;
            params.noise_mag[i] = 0.0f;
            params.noise_gps[i] = 0.0;
        }
        params.noise_baro = 0.0f;
        params.gps_origin[0] = params.gps_origin[1] = params.gps_origin[2] = 0.0f;
    }
};

// No global instances: use handle-based API only

// ============================================================================
// New Handle-based API (supports multiple instances)
// ============================================================================

FilterHandle filter_create(FilterType type) {
    FilterInstance* inst = new FilterInstance();
    Filter* f = nullptr;
    if (type == FILTER_ESKF) {
        f = new ESKFFilter();
    } else {
        f = new ESKFFilter();  // default to ESKF for unsupported types
    }
    inst->impl = f;
    return reinterpret_cast<FilterHandle>(inst);
}

void filter_destroy(FilterHandle h) {
    if (!h) return;
    FilterInstance* inst = reinterpret_cast<FilterInstance*>(h);
    if (inst->impl) delete inst->impl;
    delete inst;
}

uint8_t filter_init(FilterHandle h, const SensorData* init_data, uint32_t /*init_samples*/, float dt) {
    if (!h) return 1;
    FilterInstance* inst = reinterpret_cast<FilterInstance*>(h);
    if (!inst->impl) return 1;

    SensorData tmp;
    if (init_data) tmp = *init_data; else std::memset(&tmp, 0, sizeof(tmp));

    float static_time = dt > 0.0f ? dt : 5.0f;
    uint8_t r = inst->impl->init(tmp, static_time);
    inst->initialized = (r == 0);
    return r;
}

uint8_t filter_set_params(FilterHandle h, const Params& params) {
    if (!h) return 1;
    FilterInstance* inst = reinterpret_cast<FilterInstance*>(h);
    if (!inst->impl) return 1;
    inst->params = params;
    return inst->impl->setParams(params);
}

uint8_t filter_set_gps_origin(FilterHandle h, double lat, double lon, double alt) {
    if (!h) return 1;
    FilterInstance* inst = reinterpret_cast<FilterInstance*>(h);
    if (!inst->impl) return 1;
    // store gps origin in params (Params::gps_origin is float[3])
    inst->params.gps_origin[0] = static_cast<float>(lat);
    inst->params.gps_origin[1] = static_cast<float>(lon);
    inst->params.gps_origin[2] = static_cast<float>(alt);
    // propagate via setParams if implemented by the filter
    return inst->impl->setParams(inst->params);
}

uint8_t filter_update(FilterHandle h, const SensorData& obs) {
    if (!h) return 1;
    FilterInstance* inst = reinterpret_cast<FilterInstance*>(h);
    if (!inst->impl) return 1;
    return inst->impl->update(obs);
}

uint8_t filter_get_state(FilterHandle h, State& out) {
    if (!h) return 1;
    FilterInstance* inst = reinterpret_cast<FilterInstance*>(h);
    if (!inst->impl) return 1;
    return inst->impl->getState(out);
}

uint8_t filter_reset(FilterHandle h) {
    if (!h) return 1;
    FilterInstance* inst = reinterpret_cast<FilterInstance*>(h);
    if (!inst->impl) return 1;
    uint8_t r = inst->impl->reset();
    inst->initialized = false;
    return r;
}

uint8_t filter_is_initialized(FilterHandle h) {
    if (!h) return 0;
    FilterInstance* inst = reinterpret_cast<FilterInstance*>(h);
    return inst->initialized ? 1 : 0;
}

const char* filter_get_version(void) {
    return "kalman_filter_v2_unified";
}

} // namespace kalman
