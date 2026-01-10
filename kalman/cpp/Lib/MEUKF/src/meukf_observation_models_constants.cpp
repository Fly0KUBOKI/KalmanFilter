// Out-of-class definitions for static const arrays declared in the header.
#include "../inc/meukf_observation_models.hpp"

namespace meukf {

const float AccelObservationModel::g_ned[3] = { 0.0f, 0.0f, 9.81f };

const float MagObservationModel::m_ned[3] = { 1.0f, 0.0f, 0.2f };

} // namespace meukf
