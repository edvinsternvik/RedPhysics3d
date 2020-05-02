#include "CollisionBox.h"
#include <cmath>

namespace redPhysics3d {

    CollisionBox::CollisionBox() : m_size(1.0, 1.0, 1.0) {
        verticies = getBoxVerticies();
        updateAABBsize();
    }

    void CollisionBox::updateAABBsize() {
        AABBmin = AABBmax = verticies[0];
        for(int i = 1; i < 8; ++i) {
            if(verticies[i].x < AABBmin.x) AABBmin.x = verticies[i].x;
            if(verticies[i].y < AABBmin.y) AABBmin.y = verticies[i].y;
            if(verticies[i].z < AABBmin.z) AABBmin.z = verticies[i].z;
            if(verticies[i].x > AABBmax.x) AABBmax.x = verticies[i].x;
            if(verticies[i].y > AABBmax.y) AABBmax.y = verticies[i].y;
            if(verticies[i].z > AABBmax.z) AABBmax.z = verticies[i].z;
        }
    }

    void CollisionBox::setSize(const Vector3& newSize) {
        m_size = newSize;
        verticies = getBoxVerticies();
        updateAABBsize();
    }

    std::array<Vector3, 8> CollisionBox::getBoxVerticies() {
        Vector3 hs = m_size; // * 0.5
        return { Vector3(hs.x,  hs.y, hs.z), Vector3(hs.x,  hs.y, -hs.z), Vector3(-hs.x,  hs.y, -hs.z), Vector3(-hs.x,  hs.y, hs.z),
                 Vector3(hs.x, -hs.y, hs.z), Vector3(hs.x, -hs.y, -hs.z), Vector3(-hs.x, -hs.y, -hs.z), Vector3(-hs.x, -hs.y, hs.z)};
    }

}