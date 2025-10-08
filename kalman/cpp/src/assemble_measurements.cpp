#include "../inc/kalman_filter.hpp"

// Minimal AssembleMeasurements implementation supporting gps (2), vel (2)
// Fills out_z, out_h, out_H (row-major z_len x state_size), out_R (row-major z_len x z_len), tags
// Assumes out arrays are preallocated to OBS_SIZE_MAX and state_size provided

void KalmanFilter::AssembleMeasurements(const Meas& meas, const float* x_pred, float* out_z, float* out_h,
                                       float* out_H, float* out_R, MeasTag* tags, uint8_t& z_len, uint8_t state_size) {
    // initialize
    for (uint8_t i = 0; i < OBS_SIZE_MAX * state_size; ++i) out_H[i] = 0.0f;
    for (uint8_t i = 0; i < OBS_SIZE_MAX * OBS_SIZE_MAX; ++i) out_R[i] = 0.0f;
    z_len = 0;

    auto add_block = [&](const float* zb, uint8_t zb_len, const float* hb, const float* Hb_rowmajor, const float* Rb_diag, const char* name) {
        uint8_t start = z_len;
        // append z and h
        for (uint8_t i = 0; i < zb_len; ++i) {
            out_z[start + i] = zb[i];
            out_h[start + i] = hb[i];
        }
        // append H (Hb_rowmajor is zb_len x state_size)
        for (uint8_t r = 0; r < zb_len; ++r) {
            for (uint8_t c = 0; c < state_size; ++c) {
                out_H[(start + r) * state_size + c] = Hb_rowmajor[r * state_size + c];
            }
        }
        // append R diag (Rb_diag is length zb_len)
        for (uint8_t r = 0; r < zb_len; ++r) {
            out_R[(start + r) * OBS_SIZE_MAX + (start + r)] = Rb_diag[r];
        }
        // tag
        strncpy(tags[start].name, name, 15); tags[start].name[15] = '\0';
        tags[start].start = start;
        tags[start].length = zb_len;
        z_len += zb_len;
    };

    // gps
    if (meas.has_gps) {
        float zg[2] = { meas.gps[0], meas.gps[1] };
        float hg[2] = { x_pred[0], x_pred[1] };
        float Hg_row[2 * STATE_SIZE_MAX];
        for (uint8_t i = 0; i < 2 * state_size; ++i) Hg_row[i] = 0.0f;
        if (state_size >= 2) { Hg_row[0 * state_size + 0] = 1.0f; Hg_row[1 * state_size + 1] = 1.0f; }
        float Rg_diag[2] = { meas.gps[0] * 0.0f + 1.0f, meas.gps[0] * 0.0f + 1.0f }; // minimal default -> 1.0
        add_block(zg, 2, hg, Hg_row, Rg_diag, "gps");
    }
    // vel
    if (meas.has_vel) {
        float zv[2] = { meas.vel[0], meas.vel[1] };
        float hv[2] = { 0.0f, 0.0f };
        if (state_size >= 4) { hv[0] = x_pred[2]; hv[1] = x_pred[3]; }
        float Hv_row[2 * STATE_SIZE_MAX];
        for (uint8_t i = 0; i < 2 * state_size; ++i) Hv_row[i] = 0.0f;
        if (state_size >= 4) { Hv_row[0 * state_size + 2] = 1.0f; Hv_row[1 * state_size + 3] = 1.0f; }
        float Rv_diag[2] = { 0.1f, 0.1f };
        add_block(zv, 2, hv, Hv_row, Rv_diag, "vel");
    }
    // baro removed

    // zero-fill remaining z/h if any
    for (uint8_t i = z_len; i < OBS_SIZE_MAX; ++i) { out_z[i] = 0.0f; out_h[i] = 0.0f; }
}
