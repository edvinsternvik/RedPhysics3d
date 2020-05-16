#include "CollisionResponse.h"

namespace redPhysics3d {

    void CollisionResponse::collisionResponse(const CollisionData& collisionData) {
        Vector3 depth = collisionData.collider1Normal * collisionData.depth * 1.001;
        collisionData.collider1->setPosition(collisionData.collider1->getPosition() + depth * 0.5);
        collisionData.collider2->setPosition(collisionData.collider2->getPosition() - depth * 0.5);
    }

}