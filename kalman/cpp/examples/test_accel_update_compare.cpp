#include <cstdio>
#include <cmath>
#include "../Lib/MEUKF/inc/meukf_core.hpp"

using namespace meukf;

int main() {
    // Build initial state (identity quaternion)
    State s_init;
    for (int i=0;i<3;i++) s_init.p[i]=0.0f;
    for (int i=0;i<3;i++) s_init.v[i]=0.0f;
    s_init.q[0]=1.0f; s_init.q[1]=0.0f; s_init.q[2]=0.0f; s_init.q[3]=0.0f;
    for (int i=0;i<3;i++) s_init.ba[i]=0.0f;
    for (int i=0;i<3;i++) s_init.bg[i]=0.0f;
    for (int i=0;i<15*15;i++) s_init.P[i] = 0.0f;
    for (int i=0;i<15;i++) s_init.P[i*15 + i] = 0.01f; // small covariance

    // Sensor
    SensorData sensor{};
    sensor.accel[0]=0.0f; sensor.accel[1]=0.0f; sensor.accel[2]=-9.81f;
    sensor.update_accel = 1;

    // Params
    Params p;
    p.g[0]=0.0f; p.g[1]=0.0f; p.g[2]=-9.81f;
    for (int i=0;i<3;i++) p.noise_accel[i]=0.01f;
    p.alpha = 1e-3f; p.beta = 2.0f; p.kappa = 0.0f;

    MEUKFOutput out_orig, out_ukf;
    MEUKFCore::compare_accel_updates(s_init, sensor, p, out_orig, out_ukf);

    // Compare quaternion
    printf("Original q: %.6f %.6f %.6f %.6f\n", out_orig.new_state.q[0], out_orig.new_state.q[1], out_orig.new_state.q[2], out_orig.new_state.q[3]);
    printf("UKF q:      %.6f %.6f %.6f %.6f\n", out_ukf.new_state.q[0], out_ukf.new_state.q[1], out_ukf.new_state.q[2], out_ukf.new_state.q[3]);

    // Compare attitude covariances (3x3 block)
    printf("Original P_att:\n");
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++) printf(" %.6f", out_orig.new_state.P[(6+i)*15 + (6+j)]);
        printf("\n");
    }
    printf("UKF P_att:\n");
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++) printf(" %.6f", out_ukf.new_state.P[(6+i)*15 + (6+j)]);
        printf("\n");
    }

    // Print innovation y if available
    printf("Original last_y_len=%d, UKF last_y_len=%d\n", out_orig.last_y_len, out_ukf.last_y_len);
    if (out_orig.last_y_len>0) printf("Original y: %.6f %.6f %.6f\n", out_orig.last_y[0], out_orig.last_y[1], out_orig.last_y[2]);
    if (out_ukf.last_y_len>0) printf("UKF y:      %.6f %.6f %.6f\n", out_ukf.last_y[0], out_ukf.last_y[1], out_ukf.last_y[2]);

    return 0;
}
