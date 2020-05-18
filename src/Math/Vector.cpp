#include "Vector.h"
#include <cmath>

namespace redPhysics3d {

    Vector2::Vector2(float x, float y) : x(x), y(y) {
    }

    Vector2::Vector2() : x(0), y(0) {
    }

    Vector3::Vector3(const float& x, const float& y, const float& z) : x(x), y(y), z(z) {
    }

    Vector3::Vector3() : x(0), y(0), z(0) {
    }

    float Vector3::dot(const Vector3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    Vector3 Vector3::cross(const Vector3& other) const {
        return Vector3(y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x);
    }

    float Vector3::magnitude() const {
        return std::sqrt(x*x + y*y + z*z);
    }

    float Vector3::magnitudeSquare() const {
        return x*x + y*y + z*z;
    }

    void Vector3::normalize() {
        double invSqrt = 1.0000 / std::sqrt(x*x + y*y + z*z);
        x *= invSqrt;
        y *= invSqrt;
        z *= invSqrt;
    }
}