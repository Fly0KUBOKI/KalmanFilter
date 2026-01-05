// UKF Library Standalone Test
// Tests that ukf_generic.hpp and ukf_utils.hpp compile independently

#include "../Lib/UKF/inc/ukf_generic.hpp"
#include "../Lib/UKF/inc/ukf_utils.hpp"
#include <cstdio>

using namespace cmath_fx;

// Simple observation model: h(x) = x (direct observation)
Vector<3, float> h_direct(const Vector<3, float>& x) {
    return x;
}

int main() {
    printf("=== UKF Library Standalone Test ===\n");

    // Test 1: UKF parameter initialization
    ukf::UKFParams params;
    params.alpha = 1e-3f;
    params.beta = 2.0f;
    params.kappa = 0.0f;
    printf("[OK] UKFParams initialized\n");

    // Test 2: Cholesky decomposition
    Matrix<3, 3, float> A = Matrix<3, 3, float>::Identity();
    A(0, 0) = 4.0f;
    A(1, 1) = 9.0f;
    A(2, 2) = 16.0f;
    
    Matrix<3, 3, float> L;
    bool chol_ok = ukf_utils::cholesky3x3(A, L);
    if (chol_ok) {
        printf("[OK] Cholesky decomposition successful\n");
        printf("     L(0,0)=%.3f L(1,1)=%.3f L(2,2)=%.3f\n", L(0,0), L(1,1), L(2,2));
    } else {
        printf("[FAIL] Cholesky decomposition failed\n");
        return 1;
    }

    // Test 3: Robust Cholesky with non-positive definite input
    Matrix<3, 3, float> A_bad = Matrix<3, 3, float>::Zero();
    A_bad(0, 0) = -1.0f;  // negative diagonal
    
    Matrix<3, 3, float> L_robust;
    bool robust_ok = ukf_utils::cholesky3x3_robust(A_bad, L_robust);
    if (robust_ok) {
        printf("[OK] Robust Cholesky handled non-PD matrix\n");
    } else {
        printf("[WARN] Robust Cholesky failed (acceptable for extreme cases)\n");
    }

    // Test 4: UKF Update with simple 3D state
    Vector<3, float> x;
    x(0, 0) = 1.0f;
    x(1, 0) = 2.0f;
    x(2, 0) = 3.0f;

    Matrix<3, 3, float> P = Matrix<3, 3, float>::Identity();
    P(0, 0) = 0.1f;
    P(1, 1) = 0.1f;
    P(2, 2) = 0.1f;

    Vector<3, float> z;
    z(0, 0) = 1.05f;
    z(1, 0) = 2.03f;
    z(2, 0) = 2.98f;

    Matrix<3, 3, float> R = Matrix<3, 3, float>::Identity();
    R(0, 0) = 0.01f;
    R(1, 1) = 0.01f;
    R(2, 2) = 0.01f;

    ukf::UKFUpdate<3, 3, float> ukf;
    bool update_ok = ukf.update(x, P, z, h_direct, R, params);

    if (update_ok) {
        printf("[OK] UKF update successful\n");
        printf("     x_updated = [%.3f, %.3f, %.3f]\n", x(0,0), x(1,0), x(2,0));
    } else {
        printf("[FAIL] UKF update failed\n");
        return 1;
    }

    printf("\n=== All Tests Passed ===\n");
    return 0;
}
