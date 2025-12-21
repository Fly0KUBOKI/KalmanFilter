// compare_cholesky.cpp
// Compare Eigen::LLT result with a naive Cholesky implementation
// Build: g++ -O3 -I path/to/eigen -std=c++17 compare_cholesky.cpp -o compare_cholesky.exe

#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <vector>
#include <cmath>
#include <iomanip>

// Naive Cholesky (lower-triangular) on dense matrix stored as Eigen::MatrixXd
// Returns true on success and writes L (lower-triangular) such that A ≈ L * L.transpose()
bool naive_cholesky(const Eigen::MatrixXd &A, Eigen::MatrixXd &L, double eps = 1e-12) {
    const int n = (int)A.rows();
    L = Eigen::MatrixXd::Zero(n, n);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j <= i; ++j) {
            double sum = 0.0;
            for (int k = 0; k < j; ++k) sum += L(i, k) * L(j, k);
            if (i == j) {
                double diag = A(i, i) - sum;
                if (diag <= eps) return false;
                L(i, j) = std::sqrt(diag);
            } else {
                if (std::abs(L(j, j)) < eps) return false;
                L(i, j) = (A(i, j) - sum) / L(j, j);
            }
        }
    }
    return true;
}

double frobenius_norm(const Eigen::MatrixXd &M) {
    return std::sqrt((M.array() * M.array()).sum());
}

int main(int argc, char **argv) {
    const int n = 15;               // dimension used in UKF (example)
    const int trials = 50;
    std::mt19937_64 rng(12345);
    std::normal_distribution<double> nd(0.0, 1.0);

    double max_rel_err = 0.0;
    double sum_rel_err = 0.0;
    int failures = 0;

    for (int t = 0; t < trials; ++t) {
        // Create random SPD matrix: A = M^T * M + alpha * I
        Eigen::MatrixXd M(n, n);
        for (int i = 0; i < n; ++i) for (int j = 0; j < n; ++j) M(i, j) = nd(rng);
        Eigen::MatrixXd A = M.transpose() * M;
        double alpha = 1e-6;
        A.diagonal().array() += alpha;

        // Eigen reference
        Eigen::LLT<Eigen::MatrixXd> llt(A);
        if (llt.info() != Eigen::Success) {
            std::cerr << "Eigen LLT failed on trial " << t << "\n";
            ++failures;
            continue;
        }
        Eigen::MatrixXd L_e = llt.matrixL();

        // Naive cholesky
        Eigen::MatrixXd L_n;
        bool ok = naive_cholesky(A, L_n, 1e-14);
        if (!ok) {
            std::cerr << "Naive Cholesky failed on trial " << t << "\n";
            ++failures;
            continue;
        }

        // Compare L matrices (note: sign/ordering may differ, compare A reconstructions)
        Eigen::MatrixXd A_e = L_e * L_e.transpose();
        Eigen::MatrixXd A_n = L_n * L_n.transpose();
        Eigen::MatrixXd D = A_e - A_n;

        double rel_err = frobenius_norm(D) / (1e-12 + frobenius_norm(A_e));
        sum_rel_err += rel_err;
        if (rel_err > max_rel_err) max_rel_err = rel_err;
    }

    double avg_rel_err = sum_rel_err / (double)(trials - failures);
    std::cout << std::fixed << std::setprecision(8);
    std::cout << "Trials: " << trials << ", Failures: " << failures << "\n";
    std::cout << "Avg relative Frobenius error: " << avg_rel_err << "\n";
    std::cout << "Max relative Frobenius error: " << max_rel_err << "\n";

    if (avg_rel_err > 1e-8) {
        std::cout << "Note: naive Cholesky shows non-negligible deviation from Eigen.\n";
    } else {
        std::cout << "Naive Cholesky closely matches Eigen.\n";
    }

    return 0;
}
