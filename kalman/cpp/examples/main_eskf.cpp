#include "../Inc/kalman_all.hpp"
#include <iostream>

int main() {
    std::cout << "kalman example (minimal) - building against kalman_lib" << std::endl;
    // Minimal smoke test: include the master header. Real example should construct
    // an ESKF/UKF instance and run a short step.
    return 0;
}
