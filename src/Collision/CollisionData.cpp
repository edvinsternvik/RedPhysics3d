#include "CollisionData.h"

namespace redPhysics3d {

    CollisionData::CollisionData(bool collided, CollisionShape* collider1, CollisionShape* collider2)
        : collided(collided), collider1(collider1), collider2(collider2) {
    }

    void CollisionData::setCollisionData(const Vector3& collider1Normal, const float& depth) {
        this->collided = true;
        this->collider1Normal = collider1Normal;
        this->depth = depth;
    }

    void CollisionData::addContactPoint(const Vector3& point) {
        contactPoints.push_back(point);
    }


}