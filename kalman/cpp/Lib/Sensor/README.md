# Sensor Module - Consolidated Structure

## Overview
The Sensor module has been refactored from 11 files to **5 core files** (54% reduction), eliminating redundant implementations and establishing a unified namespace hierarchy.

## File Structure

```
Sensor/
├── coordinate_transform.hpp    (sensor::coord::)
├── sensor_processing.hpp      (sensor::processing::)  
├── sensor_preprocessor.hpp    (sensor::preprocess::)
├── sensor_filters.hpp         (sensor::filter::)
└── sensor_all.hpp             (Aggregator)
```

## Functional Layers

### 1. Coordinate Transforms (`sensor::coord::`)
**File**: `coordinate_transform.hpp`

- **GPS LLA ↔ ENU conversion**
  - `lla_to_enu_simple()` - GPS to local ENU frame
  - `enu_to_lla_simple()` - ENU to GPS coordinates
  
- **Pressure to altitude**
  - `pressure_to_altitude_d()` - Barometric altitude (double precision)
  - `pressure_to_altitude_f()` - Barometric altitude (float precision)

### 2. Sensor Signal Processing (`sensor::processing::`)
**File**: `sensor_processing.hpp`

- **Accelerometer processing**
  - `accel_to_quaternion()` - Estimate Roll/Pitch from gravity
  - `accel_to_euler()` - Compute Euler angles
  
- **Magnetometer processing**
  - `mag_observation_prediction()` - Predict magnetic field in body frame
  - `compute_mag_heading()` - Extract yaw from magnetometer

### 3. Sensor Preprocessing (`sensor::preprocess::`)
**File**: `sensor_preprocessor.hpp`

- **Raw sensor buffering**: Detect if sensor value changed significantly
- **Simple outlier flagging**: Basic threshold-based rejection
- **Return type**: `sensor::preprocess::Result { output, is_outlier, no_change }`

**Functions**:
- `accel()` - Preprocess accelerometer
- `mag()` - Preprocess magnetometer
- `baro()` - Preprocess barometer (delegates to `coord::pressure_to_altitude_d`)
- `gps()` - Preprocess GPS (delegates to `coord::lla_to_enu_simple`)

### 4. Sensor Filtering (`sensor::filter::`)
**File**: `sensor_filters.hpp`

Unified filtering library consolidating **7 previously-separate files**:

#### Filter Classes
- **`EMAFilter`** - Exponential Moving Average
- **`AlphaBetaFilter`** - Alpha-Beta tracking filter
- **`BiquadLowpassFilter`** - 2nd order IIR lowpass

#### Statistical Tools
- **`OutlierDetector`** - Mahalanobis distance-based rejection
- **`NoiseEstimator`** - Adaptive R matrix estimation
- **`DivergenceGuard`** - Covariance regularization

#### Unified Interface
- **`SensorFilterLib`** - Complete sensor filtering pipeline
  - `filter_accel()` - Accelerometer outlier detection + filtering
  - `filter_mag()` - Magnetometer outlier detection + filtering
  - `filter_baro()` - Barometer outlier detection + filtering
  - `filter_gps()` - GPS outlier detection + filtering

## Namespace Hierarchy

```cpp
sensor::
├── coord::          // Coordinate transforms
├── processing::     // Signal processing (quaternion, Euler)
├── preprocess::     // Raw sensor preprocessing
└── filter::         // Filtering, outlier detection, noise estimation
```

### Backward Compatibility

For legacy code, type aliases are provided:
```cpp
namespace common::sensor {
    using EMAFilter = sensor::filter::EMAFilter;
    using OutlierDetector = sensor::filter::OutlierDetector;
    // ... all classes aliased
}
```

## Usage Examples

### Include Everything
```cpp
#include "Sensor/sensor_all.hpp"
```

### Coordinate Transform
```cpp
double enu[3];
sensor::coord::lla_to_enu_simple(lat, lon, alt, lat0, lon0, alt0, enu);
double altitude = sensor::coord::pressure_to_altitude_d(pressure);
```

### Sensor Preprocessing
```cpp
auto result = sensor::preprocess::accel(a_meas, prev_a, tolerance);
if (!result.is_outlier && !result.no_change) {
    // Use result.output
}
```

### Sensor Signal Processing
```cpp
float q_out[4];
sensor::processing::accel_to_quaternion(a_meas, dt, q_out);
sensor::processing::mag_observation_prediction(q, m_world, m_body_expected);
```

### Sensor Filtering
```cpp
sensor::filter::SensorFilterLib filter_lib;
bool is_outlier;
auto filtered_accel = filter_lib.filter_accel(a_meas, a_expected, is_outlier);
if (!is_outlier) {
    // Use filtered_accel
}
```

## Migration from Old Structure

### Deleted Files (7 total)
Previously scattered implementations:
- ❌ `ema_filter.hpp`
- ❌ `alpha_beta_filter.hpp`
- ❌ `biquad_filter.hpp`
- ❌ `outlier_detector.hpp`
- ❌ `robust_statistics.hpp`
- ❌ `filters.hpp`
- ❌ `sensor_filter.hpp`
- ❌ `sensor_preprocessor.cpp` (moved to header-only)

### Include Path Updates
**Old**:
```cpp
#include "Sensor/sensor_filter.hpp"
#include "Sensor/outlier_detector.hpp"
```

**New**:
```cpp
#include "Sensor/sensor_all.hpp"  // Single include
// OR specific:
#include "Sensor/sensor_filters.hpp"
```

### Namespace Updates
**Old**:
```cpp
common::sensor::SensorFilterLib filter_lib;
PreprocessResult result = preprocess_accel(...);
```

**New**:
```cpp
sensor::filter::SensorFilterLib filter_lib;
sensor::preprocess::Result result = sensor::preprocess::accel(...);
```

## Design Principles

1. **Header-only**: All implementations inline for template flexibility
2. **Namespace isolation**: Clear separation by functional area
3. **Single aggregator**: `sensor_all.hpp` for convenience
4. **Backward compatibility**: Type aliases for gradual migration
5. **No redundancy**: Consolidated similar implementations

## Build Verification

After refactoring, MEX binary size:
- **Before**: ~150 KB (multiple inclusions, code bloat)
- **After**: 149 KB (consolidated, optimized)

Compilation successful with MSVC 2022.

## Testing

Run regression test:
```matlab
cd kalman
run_batch_10sets();  % 10-seed statistical validation
```

Run single test:
```matlab
run_simulation(42, true);  % seed=42, verbose
```

---

**Refactored**: 2025-01-17  
**Status**: ✅ Complete (Build successful, 5 files, unified namespace)
