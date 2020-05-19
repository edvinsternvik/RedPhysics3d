#include "CollisionResponse.h"

#include "../CollisionBody/RigidBody.h"

namespace redPhysics3d {

    void CollisionResponse::collisionResponse(const CollisionData& collisionData, CollisionBody* b1, CollisionBody* b2) {
        bool b1Dynamic = b1->getCollisionBodyType() == CollisionBodyType::Dynamic, b2Dynamic = b2->getCollisionBodyType() == CollisionBodyType::Dynamic;

        RigidBody* b1Rb = b1Dynamic ? (RigidBody*)b1 : nullptr;
        RigidBody* b2Rb = b2Dynamic ? (RigidBody*)b2 : nullptr;


        Vector3 n = collisionData.collider1Normal;
        float elasticity = 1.0;

        if(collisionData.contactPoints.size() > 0) {
            Vector3 contactPoint;
            for(auto& cp : collisionData.contactPoints) {
                contactPoint += cp;
            }
            contactPoint = contactPoint / (float)collisionData.contactPoints.size();

            Vector3 r1 = contactPoint - b1->getPosition();
            Vector3 r2 = contactPoint - b2->getPosition();

            Vector3 v1 = b1Dynamic ? b1Rb->linearVelocity + b1Rb->angularVelocity.cross(r1) : Vector3(0.0,0.0,0.0);
            Vector3 v2 = b2Dynamic ? b2Rb->linearVelocity + b2Rb->angularVelocity.cross(r2) : Vector3(0.0,0.0,0.0);

            Vector3 dv = v2 - v1;

            float invMass1 = b1Dynamic ? b1Rb->getInverseMass() : 0.0;
            float invMass2 = b2Dynamic ? b2Rb->getInverseMass() : 0.0;
            Vector3 invInertia1 = b1Dynamic ? b1Rb->getInverseInertia() : Vector3(0.0,0.0,0.0);
            Vector3 invInertia2 = b2Dynamic ? b2Rb->getInverseInertia() : Vector3(0.0,0.0,0.0);

            float thing = (invMass1 + invMass2) + n.dot((invInertia1 * r1.cross(n)).cross(r1) + (invInertia2 * r2.cross(n)).cross(r2));

            if(thing > 0.0) {
                float jn = -n.dot(dv * (1.0 + elasticity));
                if(jn > 0.0) jn = 0.0;
                jn /= thing;

                if(b1Dynamic) {
                    b1Rb->linearVelocity -= n * (jn * invMass1);
                    b1Rb->angularVelocity -= invInertia1 * r1.cross(n * jn);
                }

                if(b2Dynamic) {
                    b2Rb->linearVelocity += n * (jn * invMass2);
                    b2Rb->angularVelocity += invInertia2 * r2.cross(n * jn);
                }
            }
        }

        float movementMultiplier = (b1Dynamic && b2Dynamic) ? 0.5 : 1.0;
        Vector3 depth = collisionData.collider1Normal * collisionData.depth * 1.001;

        if(b1Dynamic) b1->setPosition(b1->getPosition() + depth * movementMultiplier * 0.5);
        if(b2Dynamic) b2->setPosition(b2->getPosition() - depth * movementMultiplier * 0.5);
    }

}