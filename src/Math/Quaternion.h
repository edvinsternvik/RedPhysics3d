#pragma once
#include "Vector.h"
#include "Matrix3x3.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>
#include <ostream>

namespace redPhysics3d {

    class Quaternion {
    public:
        Quaternion() : w(1.0), x(0.0), y(0.0), z(0.0) {

        }

        Quaternion(const float& w, const float& x, const float& y, const float& z) : w(w), x(x), y(y), z(z) {

        }


        // Taken from wikipedia
        Quaternion(const float& eulerX, const float& eulerY, const float& eulerZ) {
            float hx = eulerX * 0.5, hy = eulerY * 0.5, hz = eulerZ * 0.5;
            float cy = cos(hz), sy = sin(hz);
            float cp = cos(hy), sp = sin(hy);
            float cr = cos(hx), sr = sin(hx);

            this->w = cr * cp * cy + sr * sp * sy;
            this->x = sr * cp * cy - cr * sp * sy;
            this->y = cr * sp * cy + sr * cp * sy;
            this->z = cr * cp * sy - sr * sp * cy;
        }

        Quaternion(const Vector3& vectorRotation) : Quaternion(vectorRotation.x, vectorRotation.y, vectorRotation.z) {
            
        }

        Vector3 toEuler() const {
            Vector3 eulerAngles;

            double sinr_cosp = 2 * (w * x + y * z);
            double cosr_cosp = 1 - 2 * (x * x + y * y);
            eulerAngles.x = std::atan2(sinr_cosp, cosr_cosp);

            double sinp = 2 * (w * y - z * x);
            if (std::abs(sinp) >= 1)
                eulerAngles.y = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
            else
                eulerAngles.y = std::asin(sinp);

            double siny_cosp = 2 * (w * z + x * y);
            double cosy_cosp = 1 - 2 * (y * y + z * z);
            eulerAngles.z = std::atan2(siny_cosp, cosy_cosp);

            return eulerAngles;
        }

        Quaternion inverse() const {
            return Quaternion(w, -x, -y, -z);
        }

        void normalize() {
            float d = w*w + x*x + y*y + z*z;

            if(d == 0) {
                w = 1;
                return;
            }

            d = (float)1.0/sqrt(d);
            w *= d;
            x *= d;
            y *= d;
            z *= d;
        }

        void addScaledVector(const Vector3& vector, float scale) {
            Quaternion q(0, vector.x * scale, vector.y * scale, vector.z * scale);

            q *= *this;
            w += q.w * 0.5;
            x += q.x * 0.5;
            y += q.y * 0.5;
            z += q.z * 0.5;
        }

        void rotate(const Vector3& v) {
            Quaternion q(0, v.x, v.y, v.z);
            *this *= q;
        }

        Matrix3x3 calculateRotationMatrix() {
            return Matrix3x3(
                1 - (2*y*y + 2*z*z), 2*x*y - 2*z*w, 2*x*z + 2*y*w,
                2*x*y + 2*z*w, 1 - (2*x*x + 2*z*z), 2*y*z - 2*x*w,
                2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - (2*x*x + 2*y*y)
            );
        }


        Quaternion operator*(const Quaternion& other) const {
            return Quaternion(
                (this->w * other.w) - (this->x * other.x) - (this->y * other.y) - (this->z * other.z),
                (this->x * other.w) + (this->w * other.x) + (this->y * other.z) - (this->z * other.y),
                (this->y * other.w) + (this->w * other.y) + (this->z * other.x) - (this->x * other.z),
                (this->z * other.w) + (this->w * other.z) + (this->x * other.y) - (this->y * other.x));
        }
        
        void operator*=(const Quaternion& other) {
            Quaternion q = *this;
            this->w = (q.w * other.w) - (q.x * other.x) - (q.y * other.y) - (q.z * other.z),
            this->x = (q.x * other.w) + (q.w * other.x) + (q.y * other.z) - (q.z * other.y),
            this->y = (q.y * other.w) + (q.w * other.y) + (q.z * other.x) - (q.x * other.z),
            this->z = (q.z * other.w) + (q.w * other.z) + (q.x * other.y) - (q.y * other.x);
        }

        friend std::ostream& operator<<(std::ostream& os, const Quaternion& q) {
			return os << "{ " << q.w << ", " << q.x << ", " << q.y << ", " << q.z << " }";
		}

    public:
        float w, x, y, z;
    };

}