#pragma once
#include <ostream>

namespace redPhysics3d {

    class Vector3;

    class Matrix3x3 {
    public:
        Matrix3x3();
        Matrix3x3(const float& a, const float& b, const float& c, const float& d, const float& e, const float& f, const float& g, const float& h, const float& i);

        float& at(const int& i, const int& j) { return data[i][j]; }
        const float& at(const int& i, const int& j) const { return data[i][j]; }

        float det();
        Matrix3x3 inverse();

        static Matrix3x3 getRotationMatrixX(const float& rotationX);
        static Matrix3x3 getRotationMatrixY(const float& rotationY);
        static Matrix3x3 getRotationMatrixZ(const float& rotationZ);
        static Matrix3x3 getRotationMatrix(const Vector3& rotation);

        Matrix3x3 operator*(const Matrix3x3& o) const {
			return Matrix3x3(
                at(0,0) * o.at(0,0) + at(0,1) * o.at(1,0) + at(0,2) * o.at(2,0), at(0,0) * o.at(0,1) + at(0,1) * o.at(1,1) + at(0,2) * o.at(2,1), at(0,0) * o.at(0,2) + at(0,1) * o.at(1,2) + at(0,2) * o.at(2,2),
                at(1,0) * o.at(0,0) + at(1,1) * o.at(1,0) + at(1,2) * o.at(2,0), at(1,0) * o.at(0,1) + at(1,1) * o.at(1,1) + at(1,2) * o.at(2,1), at(1,0) * o.at(0,2) + at(1,1) * o.at(1,2) + at(1,2) * o.at(2,2),
                at(2,0) * o.at(0,0) + at(2,1) * o.at(1,0) + at(2,2) * o.at(2,0), at(2,0) * o.at(0,1) + at(2,1) * o.at(1,1) + at(2,2) * o.at(2,1), at(2,0) * o.at(0,2) + at(2,1) * o.at(1,2) + at(2,2) * o.at(2,2)
            );
		}

        Matrix3x3& operator*=(const Matrix3x3& o) {
            Matrix3x3 t = *this;
            this->at(0,0) = t.at(0,0) * o.at(0,0) + t.at(0,1) * o.at(1,0) + t.at(0,2) * o.at(2,0); this->at(0,1) = t.at(0,0) * o.at(0,1) + t.at(0,1) * o.at(1,1) + t.at(0,2) * o.at(2,1); this->at(0,2) =  t.at(0,0) * o.at(0,2) + t.at(0,1) * o.at(1,2) + t.at(0,2) * o.at(2,2);
            this->at(1,0) = t.at(1,0) * o.at(0,0) + t.at(1,1) * o.at(1,0) + t.at(1,2) * o.at(2,0); this->at(1,1) = t.at(1,0) * o.at(0,1) + t.at(1,1) * o.at(1,1) + t.at(1,2) * o.at(2,1); this->at(1,2) =  t.at(1,0) * o.at(0,2) + t.at(1,1) * o.at(1,2) + t.at(1,2) * o.at(2,2);
            this->at(2,0) = t.at(2,0) * o.at(0,0) + t.at(2,1) * o.at(1,0) + t.at(2,2) * o.at(2,0); this->at(2,1) = t.at(2,0) * o.at(0,1) + t.at(2,1) * o.at(1,1) + t.at(2,2) * o.at(2,1); this->at(2,2) =  t.at(2,0) * o.at(0,2) + t.at(2,1) * o.at(1,2) + t.at(2,2) * o.at(2,2);

            return *this;
		}

        friend std::ostream& operator<<(std::ostream& os, const Matrix3x3& m) {
			return os << "{ " << m.data[0][0] << ", " << m.data[0][1] << ", " << m.data[0][2] << "\n"
                              << m.data[1][0] << ", " << m.data[1][1] << ", " << m.data[1][2] << "\n"
                              << m.data[2][0] << ", " << m.data[2][1] << ", " << m.data[2][2] << "}";
		}

    private:
        float data[3][3];
    };

}