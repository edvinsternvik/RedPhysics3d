#include "CollisionResponse.h"
#include "../CollisionBody/CollisionBody.h"

namespace redPhysics3d {

    void CollisionResponse::collisionResponse(const CollisionData& collisionData, CollisionBody* b1, CollisionBody* b2) {
        Vector3 depth = collisionData.collider1Normal * collisionData.depth * 1.001;
        b1->position = (collisionData.collider1->getPosition() + depth * 0.5);
        b2->position = (collisionData.collider2->getPosition() - depth * 0.5);

        if(collisionData.contactPoints.size() > 0) {
            Vector3 contactPoint;
            for(int i = 0; i < collisionData.contactPoints.size(); ++i) contactPoint = contactPoint + collisionData.contactPoints[i];
            contactPoint = contactPoint / (float)collisionData.contactPoints.size();
            Vector3 F = collisionData.collider1Normal * collisionData.depth;

            Vector3 C1 = contactPoint - collisionData.collider1->getPosition();
            Vector3 R1 = C1.cross(F);
            b1->rotation = (collisionData.collider1->getRotation() + R1 * 0.5);

            Vector3 C2 = contactPoint - collisionData.collider2->getPosition();
            Vector3 R2 = C2.cross(F);
            b2->rotation = (collisionData.collider2->getRotation() + R2 * -0.5);
        }
    }

}