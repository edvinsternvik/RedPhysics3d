#include "CollisionResponse.h"

#include "../CollisionBody/RigidBody.h"

namespace redPhysics3d {

    void CollisionResponse::collisionResponse(const CollisionData& collisionData, CollisionBody* b1, CollisionBody* b2) {
        bool b1Dynamic = b1->getCollisionBodyType() == CollisionBodyType::Dynamic, b2Dynamic = b2->getCollisionBodyType() == CollisionBodyType::Dynamic;

        // Vector3 depth = collisionData.collider1Normal * collisionData.depth * 1.001;
        // float movementMultiplier = (b1->getCollisionBodyType() == CollisionBodyType::Dynamic && b2->getCollisionBodyType() == CollisionBodyType::Dynamic) ? 0.5 : 1.0;

        // if(b1Dynamic) b1->position = (collisionData.collider1->getPosition() + depth * movementMultiplier);
        // if(b2Dynamic) b2->position = (collisionData.collider2->getPosition() - depth * movementMultiplier);

        Vector3 v1 = b1Dynamic ? ((RigidBody*)b1)->linearVelocity : Vector3(0.0, 0.0, 0.0);
        Vector3 v2 = b2Dynamic ? ((RigidBody*)b2)->linearVelocity : Vector3(0.0, 0.0, 0.0);
        float invMass1 = b1Dynamic ? ((RigidBody*)b1)->getInverseMass() : 0.0;
        float invMass2 = b2Dynamic ? ((RigidBody*)b2)->getInverseMass() : 0.0;
        Vector3 deltaVelocity = v1 - v2;

        float velNormalDot = deltaVelocity.dot(collisionData.collider1Normal);

        float J = (-velNormalDot * 2) / (collisionData.collider1Normal.dot(collisionData.collider1Normal) * (invMass1 + invMass2));

        if(b1Dynamic) ((RigidBody*)b1)->linearVelocity += collisionData.collider1Normal * (J * invMass1);
        if(b2Dynamic) ((RigidBody*)b2)->linearVelocity -= collisionData.collider1Normal * (J * invMass2);

    }

}