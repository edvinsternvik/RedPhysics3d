#include "Vector.h"

namespace redPhysics3d {

    Vector2::Vector2(float x, float y) : x(x), y(y) {
    }

    Vector2::Vector2() : x(0), y(0) {
    }

    Vector3::Vector3(const float& x, const float& y, const float& z) : x(x), y(y), z(z) {
    }

    Vector3::Vector3() : x(0), y(0), z(0) {
    }
}