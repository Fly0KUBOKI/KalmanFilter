#include "../inc/Validation/validation.hpp"

namespace common {
namespace validation {

const float CovarianceRegularizer::MIN_VARIANCE = 1.0e-12f;
const float CovarianceRegularizer::MAX_VARIANCE = 1.0e6f;
const float CovarianceRegularizer::MIN_EIGENVALUE = 1.0e-9f;

const float StateValidator::MAX_POSITION = 1.0e6f;
const float StateValidator::MAX_VELOCITY = 1000.0f;
const float StateValidator::MAX_ACCELERATION = 100.0f;
const float StateValidator::MAX_ANGULAR_RATE = 10.0f;

const int NoiseEstimator::WINDOW_SIZE = 50;

// (robust_statistics constants are defined in a separate translation unit)

} // namespace validation
} // namespace common
