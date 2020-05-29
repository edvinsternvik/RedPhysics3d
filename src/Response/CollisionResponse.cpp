#include "CollisionResponse.h"

#include "../CollisionBody/RigidBody.h"

#include "../PhysicsWorld.h"

namespace redPhysics3d {

    void CollisionResponse::collisionResponse(const CollisionData& collisionData, CollisionBody* b1, CollisionBody* b2) {
        bool b2Dynamic = b2->getCollisionBodyType() == CollisionBodyType::Dynamic;
        RigidBody* b1Rb = (RigidBody*)b1;

        Vector3 n = collisionData.collider1Normal;

        Vector3 depth = n * collisionData.depth * 1.01;
        if(b2Dynamic) {
            RigidBody* b2Rb = (RigidBody*)b2;
            float totalInvMass = b1Rb->getInverseMass() + b2Rb->getInverseMass();

            b1->position = (b1->position + depth * (b1Rb->getInverseMass() / totalInvMass));
            b2->position = (b2->position - depth * (b2Rb->getInverseMass() / totalInvMass));
        }
        else {
            b1->position = (b1->position + depth);
        }

        float elasticity = 0.25;

        Vector3 velocityChange1, velocityChange2, angularVelocityChange1, angularVelocityChange2;

        for(const Vector3& contactPoint : collisionData.contactPoints) {
            Vector3 r1 = contactPoint - b1->position;
            Vector3 r2 = contactPoint - b2->position;

            Vector3 closingVelocity = (b1Rb->linearVelocity + b1Rb->angularVelocity.cross(r1)) * -1.0;

            Vector3 velocityPerUnitImpulse1 = (b1Rb->inverseInertia * (r1.cross(n))).cross(r1);

            float deltaVelocity = velocityPerUnitImpulse1.dot(n);
            deltaVelocity += b1Rb->getInverseMass();

            if(b2Dynamic) {
                RigidBody* b2Rb = (RigidBody*)b2;

                closingVelocity += b2Rb->linearVelocity + b2Rb->angularVelocity.cross(r2); // Maybe add?

                Vector3 velocityPerUnitImpulse2 = (b2Rb->inverseInertia * (r2.cross(n))).cross(r2);

                deltaVelocity += velocityPerUnitImpulse2.dot(n);
                deltaVelocity += b2Rb->getInverseMass();
            }

            float cn = (-1 - elasticity) * closingVelocity.dot(n);

            Vector3 impulse = n * (cn / deltaVelocity);


            velocityChange1 -= impulse * b1Rb->getInverseMass();
            angularVelocityChange1 += b1Rb->inverseInertia * (impulse.cross(r1));

            if(b2Dynamic) {
                RigidBody* b2Rb = (RigidBody*)b2;
                velocityChange2 += impulse * b2Rb->getInverseMass();
                angularVelocityChange2 -= b2Rb->inverseInertia * (impulse.cross(r2));
            }
        }

        b1Rb->linearVelocity += velocityChange1;
        b1Rb->angularVelocity += angularVelocityChange1;

        if(b2Dynamic) {
            RigidBody* b2Rb = (RigidBody*)b2;
            b2Rb->linearVelocity += velocityChange2;
            b2Rb->angularVelocity += angularVelocityChange2;
        }

    }

}