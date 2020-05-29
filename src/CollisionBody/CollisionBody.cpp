#include "CollisionBody.h"

namespace redPhysics3d {

    CollisionBody::CollisionBody() {
        
    }

    bool CollisionBody::removeCollisionShape(CollisionShape* collisionShape) {
        for(int i = 0; i < collisionShapes.size(); ++i) {
            if(collisionShapes[i].get() == collisionShape) {
                collisionShapes.erase(collisionShapes.begin() + i);
                return true;
            }
        }

        return false;
    }

    void CollisionBody::updateRotationMatricies() {
        rotationMatrix = orientation.calculateRotationMatrix();
        invertedRotationMatrix = rotationMatrix.transpose();
    }

}