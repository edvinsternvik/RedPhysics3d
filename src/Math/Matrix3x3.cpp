#include "Matrix3x3.h"
#include <cmath>
#include "Vector.h"

namespace redPhysics3d {

    Matrix3x3::Matrix3x3() : data{0,0,0,0,0,0,0,0,0} {

    }

    Matrix3x3::Matrix3x3(const float& a, const float& b, const float& c, const float& d, const float& e, const float& f, const float& g, const float& h, const float& i)
            : data{a,b,c,d,e,f,g,h,i} {

    }

    float Matrix3x3::det() const {
        return at(0,0) * (at(1,1)*at(2,2) - at(1,2)*at(2,1)) - at(0,1) * (at(1,0)*at(2,2) - at(1,2)*at(2,0)) + at(0,2) * (at(1,0)*at(2,1) - at(1,1)*at(2,0));
    }

    Matrix3x3 Matrix3x3::inverse() const {
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

    Matrix3x3 Matrix3x3::transpose() const {
        return Matrix3x3(
              at(0,0), at(1,0), at(2,0),
              at(0,1), at(1,1), at(2,1),
              at(0,2), at(1,2), at(2,2)
        );
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

    Matrix3x3 Matrix3x3::getSkewSymmetric(const Vector3& v) {
        return Matrix3x3(
          0, -v.z, v.y,
          v.z, 0, -v.x,
          -v.y, v.x, 0
        );
    }

    Matrix3x3 Matrix3x3::operator*(const Matrix3x3& o) const {
        return Matrix3x3(
            at(0,0) * o.at(0,0) + at(0,1) * o.at(1,0) + at(0,2) * o.at(2,0), at(0,0) * o.at(0,1) + at(0,1) * o.at(1,1) + at(0,2) * o.at(2,1), at(0,0) * o.at(0,2) + at(0,1) * o.at(1,2) + at(0,2) * o.at(2,2),
            at(1,0) * o.at(0,0) + at(1,1) * o.at(1,0) + at(1,2) * o.at(2,0), at(1,0) * o.at(0,1) + at(1,1) * o.at(1,1) + at(1,2) * o.at(2,1), at(1,0) * o.at(0,2) + at(1,1) * o.at(1,2) + at(1,2) * o.at(2,2),
            at(2,0) * o.at(0,0) + at(2,1) * o.at(1,0) + at(2,2) * o.at(2,0), at(2,0) * o.at(0,1) + at(2,1) * o.at(1,1) + at(2,2) * o.at(2,1), at(2,0) * o.at(0,2) + at(2,1) * o.at(1,2) + at(2,2) * o.at(2,2)
        );
    }

    void Matrix3x3::operator*=(const Matrix3x3& o) {
        Matrix3x3 t = *this;
        this->at(0,0) = t.at(0,0) * o.at(0,0) + t.at(0,1) * o.at(1,0) + t.at(0,2) * o.at(2,0); this->at(0,1) = t.at(0,0) * o.at(0,1) + t.at(0,1) * o.at(1,1) + t.at(0,2) * o.at(2,1); this->at(0,2) =  t.at(0,0) * o.at(0,2) + t.at(0,1) * o.at(1,2) + t.at(0,2) * o.at(2,2);
        this->at(1,0) = t.at(1,0) * o.at(0,0) + t.at(1,1) * o.at(1,0) + t.at(1,2) * o.at(2,0); this->at(1,1) = t.at(1,0) * o.at(0,1) + t.at(1,1) * o.at(1,1) + t.at(1,2) * o.at(2,1); this->at(1,2) =  t.at(1,0) * o.at(0,2) + t.at(1,1) * o.at(1,2) + t.at(1,2) * o.at(2,2);
        this->at(2,0) = t.at(2,0) * o.at(0,0) + t.at(2,1) * o.at(1,0) + t.at(2,2) * o.at(2,0); this->at(2,1) = t.at(2,0) * o.at(0,1) + t.at(2,1) * o.at(1,1) + t.at(2,2) * o.at(2,1); this->at(2,2) =  t.at(2,0) * o.at(0,2) + t.at(2,1) * o.at(1,2) + t.at(2,2) * o.at(2,2);
    }

    void Matrix3x3::operator+=(const Matrix3x3& o) {
        for(int i = 0; i < 3; ++i) {
            for(int j = 0; j < 3; ++j) {
                data[i][j] += o.data[i][j];
            }
        }
    }

    void Matrix3x3::operator*=(const float& o) {
        for(int i = 0; i < 3; ++i) {
            for(int j = 0; j < 3; ++j) {
                data[i][j] *= o;
            }
        }
    }

    Vector3 Matrix3x3::operator*(const Vector3& vec3) const {
        return Vector3(at(0, 0) * vec3.x + at(0, 1) * vec3.y + at(0, 2) * vec3.z,
                        at(1, 0) * vec3.x + at(1, 1) * vec3.y + at(1, 2) * vec3.z,
                        at(2, 0) * vec3.x + at(2, 1) * vec3.y + at(2, 2) * vec3.z);
    }


}