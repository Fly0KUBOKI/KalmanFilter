#include "../kalman/cpp/Lib/Matrix/matrix_decomposition.hpp"
#include "../kalman/cpp/Lib/Matrix/matrix_inverse.hpp"
#include "../kalman/cpp/Lib/Matrix/matrix_utils.hpp"
#include <cassert>
#include <iostream>
#include <cmath>

using namespace cmath_fx;

void test_cholesky() {
    Matrix<3,3,float> A;
    A(0,0)=4.0f; A(0,1)=2.0f; A(0,2)=1.0f;
    A(1,0)=2.0f; A(1,1)=5.0f; A(1,2)=3.0f;
    A(2,0)=1.0f; A(2,1)=3.0f; A(2,2)=6.0f;

    Matrix<3,3,float> L;
    bool ok = decomp::cholesky<3>(A, L);
    assert(ok);

    auto LLt = L * L.transpose();
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) assert(std::abs(LLt(i,j)-A(i,j))<1e-5f);
    std::cout<<"test_cholesky: PASSED\n";
}

void test_inverse_3x3() {
    Matrix<3,3,float> A;
    A(0,0)=4.0f; A(0,1)=7.0f; A(0,2)=2.0f;
    A(1,0)=3.0f; A(1,1)=6.0f; A(1,2)=1.0f;
    A(2,0)=2.0f; A(2,1)=5.0f; A(2,2)=3.0f;

    Matrix<3,3,float> A_inv;
    bool ok = inv::inverse_3x3_analytic<float>(A, A_inv);
    assert(ok);

    auto I_test = A * A_inv;
    auto I_exp = Matrix<3,3,float>::Identity();
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) assert(std::abs(I_test(i,j)-I_exp(i,j))<1e-4f);
    std::cout<<"test_inverse_3x3: PASSED\n";
}

int main(){
    test_cholesky();
    test_inverse_3x3();
    return 0;
}
