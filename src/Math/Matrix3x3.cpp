#include "Matrix3x3.h"
#include <cmath>

namespace redPhysics3d {

    Matrix3x3::Matrix3x3() : data{0,0,0,0,0,0,0,0,0} {

    }

    Matrix3x3::Matrix3x3(const float& a, const float& b, const float& c, const float& d, const float& e, const float& f, const float& g, const float& h, const float& i)
            : data{a,b,c,d,e,f,g,h,i} {

    }

    float Matrix3x3::det() {
        return at(0,0) * (at(1,1)*at(2,2) - at(1,2)*at(2,1)) - at(0,1) * (at(1,0)*at(2,2) - at(1,2)*at(2,0)) + at(0,2) * (at(1,0)*at(2,1) - at(1,1)*at(2,0));
    }

    Matrix3x3 Matrix3x3::inverse() {
        float invdet = 1.0 / det();

        return Matrix3x3(
            (at(1, 1) * at(2, 2) - at(2, 1) * at(1, 2)) * invdet,
            (at(0, 2) * at(2, 1) - at(0, 1) * at(2, 2)) * invdet,
            (at(0, 1) * at(1, 2) - at(0, 2) * at(1, 1)) * invdet,
            (at(1, 2) * at(2, 0) - at(1, 0) * at(2, 2)) * invdet,
            (at(0, 0) * at(2, 2) - at(0, 2) * at(2, 0)) * invdet,
            (at(1, 0) * at(0, 2) - at(0, 0) * at(1, 2)) * invdet,
            (at(1, 0) * at(2, 1) - at(2, 0) * at(1, 1)) * invdet,
            (at(2, 0) * at(0, 1) - at(0, 0) * at(2, 1)) * invdet,
            (at(0, 0) * at(1, 1) - at(1, 0) * at(0, 1)) * invdet);
    }

    Matrix3x3 Matrix3x3::getRotationMatrixX(const float& rotationX) {
        return Matrix3x3(1.0, 0.0, 0.0,  0.0, std::cos(rotationX), -std::sin(rotationX),    0.0, std::sin(rotationX), std::cos(rotationX));
    }

    Matrix3x3 Matrix3x3::getRotationMatrixY(const float& rotationY) {
        return Matrix3x3(std::cos(rotationY), 0.0, std::sin(rotationY),     0.0, 1.0, 0.0,  -std::sin(rotationY), 0.0, std::cos(rotationY));
    }

    Matrix3x3 Matrix3x3::getRotationMatrixZ(const float& rotationZ) {
        return Matrix3x3(std::cos(rotationZ), -std::sin(rotationZ), 0.0,    std::sin(rotationZ), std::cos(rotationZ), 0.0,   0.0, 0.0, 1.0);
    }


}