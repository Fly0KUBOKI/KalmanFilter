Compare tests for Eigen vs naive implementations

Files:
- `compare_cholesky.cpp`: C++ executable that generates random SPD matrices and compares Eigen::LLT to a naive Cholesky implementation.

Build (Linux / MinGW):
```
g++ -O3 -std=c++17 -I <path-to-eigen-headers> compare_cholesky.cpp -o compare_cholesky.exe
```

Build (MSVC):
```
cl /EHsc /std:c++17 /I <path-to-eigen-headers> compare_cholesky.cpp
```

Run:
```
./compare_cholesky.exe
```

Purpose:
- Quantify numeric differences between Eigen and a simple cholesky implementation.
- Use results to prioritize improvements (Kahan summation, pivoting, Jacobi eigen, blocked matmul).
