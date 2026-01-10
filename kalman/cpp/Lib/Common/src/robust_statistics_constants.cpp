#include "../inc/Sensor/robust_statistics.hpp"

// Provide explicit definitions for static members to satisfy MSVC linker
// (some MSVC versions require an out-of-class definition even for constexpr class members)
const float NoiseEstimator::R_ABS_MIN = 1e-12f;
const float NoiseEstimator::R_ABS_MAX = 1e6f;
const float NoiseEstimator::OUTLIER_FACTOR = 20.0f;

// DivergenceGuard::MAX_SENSORS is a compile-time constexpr in the header and needs no out-of-class definition

// No additional namespaced definitions are provided here to avoid
// mismatches with header declarations. The three non-namespaced
// constants above cover the required out-of-class definitions.
