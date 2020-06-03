#include "CollisionData.h"

namespace redPhysics3d {

    CollisionData::CollisionData() {
    }

    void CollisionData::addContactPoint(CollisionShape* collider1, CollisionShape* collider2, const Vector3& point, const Vector3& collider1Normal, const float& penetration) {
        contacts.push_back(Contact(collider1, collider2, point, collider1Normal, penetration));
    }


}