#include "../inc/meukf_core.hpp"
#include <cstdlib>

namespace meukf {

void MEUKFCore::step(const MEUKFInput& input, MEUKFOutput& output) {
    // Initialize output
    output.new_state = input.prev_state;
    output.status = 0;
    for(int i=0; i<10; ++i) output.debug_info[i] = 0.0;
    // Initialize optional debug outputs
    for(int i=0;i<15*3;++i) output.last_K[i] = 0.0f;
    for(int i=0;i<3;++i) output.last_y[i] = 0.0f;
    output.last_y_len = 0;
    output.last_sensor_type = 0;
    for(int i=0;i<15*15;++i) output.pred_P[i] = 0.0f;

    // 1. Prediction Step
    if (input.sensor.dt > 0.0) {
        predict(output.new_state, input.sensor, input.params);
    }

    // Capture predicted covariance (P) immediately after predict() and before any updates
    for(int i=0;i<15*15;++i) {
        output.pred_P[i] = output.new_state.P[i];
    }

    // 2. Update Step
    if (input.sensor.update_accel) {
        Vector3 a_meas = Vector3();
        a_meas(0,0) = input.sensor.accel[0]; a_meas(1,0) = input.sensor.accel[1]; a_meas(2,0) = input.sensor.accel[2];
        update_accel_meukf_ukf_version(output.new_state, a_meas, input.params, output);
    }

    if (input.sensor.update_mag) {
        Vector3 m_meas = Vector3();
        m_meas(0,0) = input.sensor.mag[0]; m_meas(1,0) = input.sensor.mag[1]; m_meas(2,0) = input.sensor.mag[2];
        update_mag_meukf_ukf_version(output.new_state, m_meas, input.params, output);
    }

    if (input.sensor.update_gps) {
        Vector3 gps_meas = Vector3();
        gps_meas(0,0) = input.sensor.gps_pos[0]; gps_meas(1,0) = input.sensor.gps_pos[1]; gps_meas(2,0) = input.sensor.gps_pos[2];
        update_gps_meukf_ukf_version(output.new_state, gps_meas, input.params, output);
    }

    if (input.sensor.update_baro) {
        float alt_baro = input.sensor.alt_baro;
        update_baro_meukf_ukf_version(output.new_state, alt_baro, input.params, output);
    }

    if (input.sensor.update_zupt) {
        update_zupt_meukf_ukf_version(output.new_state, input.params, output);
    }
}

} // namespace meukf
