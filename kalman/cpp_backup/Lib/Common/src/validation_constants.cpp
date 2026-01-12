#include "../inc/Validation/validation.hpp"

namespace common {
namespace validation {

// Covariance constants (migrated from CovarianceRegularizer)
namespace covariance {
	const float MIN_VARIANCE = 1.0e-12f;
	const float MAX_VARIANCE = 1.0e6f;
	const float MIN_EIGENVALUE = 1.0e-9f;
}

// State limits (migrated from StateValidator)
namespace state {
	const float MAX_POSITION = 1.0e6f;
	const float MAX_VELOCITY = 1000.0f;
	const float MAX_ACCELERATION = 100.0f;
	const float MAX_ANGULAR_RATE = 10.0f;
}

const int NoiseEstimator::WINDOW_SIZE = 50;

// (robust_statistics constants are defined in a separate translation unit)

} // namespace validation
} // namespace common
