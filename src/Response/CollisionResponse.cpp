#include "CollisionResponse.h"

namespace redPhysics3d {

    void CollisionResponse::collisionResponse(const CollisionData& collisionData) {
        Vector3 depth = collisionData.collider1Normal * collisionData.depth * 1.001;
        collisionData.collider1->setPosition(collisionData.collider1->getPosition() + depth * 0.5);
        collisionData.collider2->setPosition(collisionData.collider2->getPosition() - depth * 0.5);

        if(collisionData.contactPoints.size() > 0) {
            Vector3 contactPoint;
            for(int i = 0; i < collisionData.contactPoints.size(); ++i) contactPoint = contactPoint + collisionData.contactPoints[i];
            contactPoint = contactPoint / (float)collisionData.contactPoints.size();
            Vector3 F = collisionData.collider1Normal * collisionData.depth;

            Vector3 C1 = contactPoint - collisionData.collider1->getPosition();
            Vector3 R1 = C1.cross(F);
            collisionData.collider1->setRotation(collisionData.collider1->getRotation() + R1 * 0.5);

            Vector3 C2 = contactPoint - collisionData.collider2->getPosition();
            Vector3 R2 = C2.cross(F);
            collisionData.collider2->setRotation(collisionData.collider2->getRotation() + R2 * 0.5);
        }
    }

}