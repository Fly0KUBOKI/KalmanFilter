# Revert Report: MATLAB Implementation Restoration

## Overview
The attitude estimation logic has been reverted from C++ MEX to pure MATLAB implementation as requested. The C++ files remain in the project but are no longer called by the main MATLAB scripts.

## Changes Made

### 1. QuaternionLib.m
- Removed all calls to `mex_quaternion_lib`.
- Restored pure MATLAB implementation for all quaternion operations.

### 2. ukf_update.m
- Removed `mex_ukf_update` call.
- Restored pure MATLAB implementation for UKF update step.

### 3. eskf_core_mex.m
- Disabled MEX detection (`use_mex = false`).
- Implemented `predict_covariance_matlab` in pure MATLAB.
  - **Improvement**: Added the rotation term (`eye(3) - skew(w)*dt`) to the error state transition matrix `F`, which was missing in the C++ implementation. This should improve attitude estimation stability during dynamic motion.
- Ensured all other functions fall back to their MATLAB equivalents (`integrate_nominal`, `ESKFErrorInjection`, etc.).

## Verification
- Ran `run_simulation.m` successfully.
- Execution time: ~8.14s (Pure MATLAB).

## Notes
- The C++ implementation of `predict_covariance` (in `eskf_core.cpp`) was found to be missing the attitude error dynamics term in the Jacobian matrix. This likely caused the estimation collapse.
- The restored MATLAB implementation includes this term.
