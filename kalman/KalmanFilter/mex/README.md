This folder contains the MATLAB/MEX wrapper and helper scripts for the C++ Kalman filter.

Files:
- kalman_mex.cpp      - MEX wrapper (compile with mex)
- kalman_mex_build.m  - helper to build the MEX from this folder
- kalman_mex_test.m   - simple test harness
- kalman_cpp_wrapper.m - convenient MATLAB wrapper function

Build (from this folder in MATLAB):
    kalman_mex_build

Run test:
    kalman_mex_test
