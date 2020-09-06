#pragma once
#include "CollisionShapes/CollisionShape.h"
#include <vector>

namespace redPhysics3d {

    class Contact {
    public:
        Contact(CollisionShape* collider1, CollisionShape* collider2, const Vector3& point, const Vector3& collider1Normal, const float& penetration);

        void prepareContact();

    private:
        Matrix3x3 calculateWorldToContactMatrix() const;

    public:
        CollisionShape* colliders[2];
        Vector3 contactPoint;
        Vector3 collider1Normal;
        float penetration;
        Vector3 relativeContactPosition[2];
        Matrix3x3 worldToContactMatrix;
    };

    class CollisionData {
    public:
        CollisionData();

        void addContactPoint(CollisionShape* collider1, CollisionShape* collider2, const Vector3& point, const Vector3& collider1Normal, const float& penetration);

    public:
        std::vector<Contact> contacts;
    };

}