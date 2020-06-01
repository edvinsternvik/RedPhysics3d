#pragma once
#include "CollisionShapes/CollisionShape.h"
#include <vector>

namespace redPhysics3d {

    class Contact {
    public:
        Contact(const Vector3& point, const float& penetration) : contactPoint(point), penetration(penetration) {

        }

        Vector3 contactPoint;
        float penetration;
    };

    class CollisionData {
    public:
        CollisionData(bool collided, CollisionShape* collider1, CollisionShape* collider2);

        void setCollisionData(const Vector3& collider1Normal, const float& depth);
        void addContactPoint(const Vector3& point, const float& penetration);

    public:
        bool collided;
        CollisionShape* collider1, *collider2;
        Vector3 collider1Normal;
        float depth;
        std::vector<Contact> contacts;
    };

}