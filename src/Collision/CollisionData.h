#pragma once
#include "CollisionShapes/CollisionShape.h"
#include <vector>

namespace redPhysics3d {

    class CollisionData {
    public:
        CollisionData(bool collided, CollisionShape* collider1, CollisionShape* collider2);

        void setCollisionData(const Vector3& collider1Normal, const float& depth);
        void addContactPoint(const Vector3& point);

    public:
        bool collided;
        CollisionShape* collider1, *collider2;
        Vector3 collider1Normal;
        float depth;
        std::vector<Vector3> contactPoints;
    };

}