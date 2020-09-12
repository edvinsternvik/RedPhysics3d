#pragma once
#include <ostream>
#include "Vector.h"

namespace redPhysics3d {

    class Vector3;

    class Matrix3x3 {
    public:
        Matrix3x3();
        Matrix3x3(const float& a, const float& b, const float& c, const float& d, const float& e, const float& f, const float& g, const float& h, const float& i);

        float& at(const int& i, const int& j) { return data[i][j]; }
        const float& at(const int& i, const int& j) const { return data[i][j]; }

        float det() const;
        Matrix3x3 inverse() const;
        Matrix3x3 transpose() const;

        static Matrix3x3 getRotationMatrixX(const float& rotationX);
        static Matrix3x3 getRotationMatrixY(const float& rotationY);
        static Matrix3x3 getRotationMatrixZ(const float& rotationZ);

        static Matrix3x3 getSkewSymmetric(const Vector3& v);

        Matrix3x3 operator*(const Matrix3x3& o) const;
        void operator*=(const Matrix3x3& o);
        void operator+=(const Matrix3x3& o);
        void operator*=(const float& o);
        Vector3 operator*(const Vector3& vec3) const;

        friend std::ostream& operator<<(std::ostream& os, const Matrix3x3& m) {
			return os << "{ " << m.data[0][0] << ", " << m.data[0][1] << ", " << m.data[0][2] << "\n"
                              << m.data[1][0] << ", " << m.data[1][1] << ", " << m.data[1][2] << "\n"
                              << m.data[2][0] << ", " << m.data[2][1] << ", " << m.data[2][2] << "}";
		}

    private:
        float data[3][3];
    };

}