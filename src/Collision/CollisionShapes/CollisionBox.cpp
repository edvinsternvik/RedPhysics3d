#include "CollisionBox.h"
#include <cmath>

#define sqrtof3 1.732051

namespace redPhysics3d {

    CollisionBox::CollisionBox() : m_size(1.0, 1.0, 1.0) {
        updateAABBsize();
        verticies = getBoxVerticies();
    }

    void CollisionBox::updateAABBsize() {
        AABBsize = std::max(std::max(m_size.x, m_size.y), m_size.z) * sqrtof3;
    }

    void CollisionBox::setSize(const Vector3& newSize) {
        m_size = newSize;
        updateAABBsize();
        verticies = getBoxVerticies();
    }

    std::array<Vector3, 8> CollisionBox::getBoxVerticies() {
        Vector3 hs = m_size; // * 0.5
        return { Vector3(hs.x,  hs.y, hs.z), Vector3(hs.x,  hs.y, -hs.z), Vector3(-hs.x,  hs.y, -hs.z), Vector3(-hs.x,  hs.y, hs.z),
                 Vector3(hs.x, -hs.y, hs.z), Vector3(hs.x, -hs.y, -hs.z), Vector3(-hs.x, -hs.y, -hs.z), Vector3(-hs.x, -hs.y, hs.z)};
    }

}