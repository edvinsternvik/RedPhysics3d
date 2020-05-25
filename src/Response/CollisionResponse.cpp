#include "CollisionResponse.h"

#include "../CollisionBody/RigidBody.h"

namespace redPhysics3d {

    void CollisionResponse::collisionResponse(const CollisionData& collisionData, CollisionBody* b1, CollisionBody* b2) {
        bool b1Dynamic = b1->getCollisionBodyType() == CollisionBodyType::Dynamic, b2Dynamic = b2->getCollisionBodyType() == CollisionBodyType::Dynamic;

        RigidBody* b1Rb = b1Dynamic ? (RigidBody*)b1 : nullptr;
        RigidBody* b2Rb = b2Dynamic ? (RigidBody*)b2 : nullptr;

        Vector3 n = collisionData.collider1Normal;
        float totalInvMass = b1Rb->getInverseMass() + b2Rb->getInverseMass();

        float elasticity = 0.50;

        for(const Vector3& contactPoint : collisionData.contactPoints) { 

            Vector3 cpRelative1 = contactPoint - b1->getPosition();
            Vector3 cpRelative2 = contactPoint - b2->getPosition();

            Vector3 angularVelocity1 = b1Rb->angularVelocity.cross(cpRelative1);
            Vector3 angularVelocity2 = b2Rb->angularVelocity.cross(cpRelative2);

            Vector3 velocity1 = (b1Rb->linearVelocity + angularVelocity1);
            Vector3 velocity2 = (b2Rb->linearVelocity + angularVelocity2);

            Vector3 contactVelocity = velocity2 - velocity1;

            float impulseForce = contactVelocity.dot(n);
            Vector3 inertia1 = (b1Rb->getInverseInertia() * cpRelative1.cross(n)).cross(cpRelative1);
            Vector3 inertia2 = (b2Rb->getInverseInertia() * cpRelative2.cross(n)).cross(cpRelative2);

            float angularEffect = (inertia1.dot(n) + inertia2.dot(n));

            float j = (-(1.0 + elasticity) * impulseForce) / (totalInvMass + angularEffect);

            Vector3 impulse = n * j;

            b1Rb->linearVelocity -= impulse * b1Rb->getInverseMass();
            b2Rb->linearVelocity += impulse * b2Rb->getInverseMass();

            b1Rb->angularVelocity += b1Rb->getInverseInertia() * cpRelative1.cross(impulse * -1.0);
            b2Rb->angularVelocity += b2Rb->getInverseInertia() * cpRelative2.cross(impulse);
        }

        Vector3 depth = n * collisionData.depth * 1.01;
        if(b1Dynamic) b1->setPosition(b1->getPosition() + depth * (b1Rb->getInverseMass() / totalInvMass));
        if(b2Dynamic) b2->setPosition(b2->getPosition() - depth * (b2Rb->getInverseMass() / totalInvMass));
    }

}