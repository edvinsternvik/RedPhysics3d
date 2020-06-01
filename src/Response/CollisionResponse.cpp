#include "CollisionResponse.h"

#include "../CollisionBody/RigidBody.h"

#include "../PhysicsWorld.h"

namespace redPhysics3d {

    CollisionResponse::CollisionResponse(const CollisionData& collisionData, CollisionBody* b1, CollisionBody* b2) : m_collisionData(collisionData), m_b1(b1), m_b2(b2) {
        m_b2Dynamic = m_b2->getCollisionBodyType() == CollisionBodyType::Dynamic;
        m_rigidbodies[0] = (RigidBody*)m_b1;
        m_rigidbodies[1] = m_b2Dynamic ? (RigidBody*)m_b2 : nullptr;
    }

    void CollisionResponse::solveCollision() {
        // Resolve interpenetration using nonlinear projection
        for(const Contact& contact : m_collisionData.contacts) {
            resolveContact(contact.contactPoint);
        }

        // Change velocities
        applyImpulses();
    }

    void CollisionResponse::resolveContact(const Vector3& contactPoint) {
        float linearMove[2] = {0.0, 0.0}, angularMove[2] = {0.0, 0.0};
        float totalInertia = 0.0, linearInertia[2] = {0.0, 0.0}, angularInertia[2] = {0.0,0.0};
        Vector3 rotationPerMove[2];
        for(int i = 0; i < 2; ++i) {
            if(m_rigidbodies[i] != nullptr) {
                Vector3 contactRelative = contactPoint - m_rigidbodies[i]->position;

                Vector3 impulsePerMove = contactRelative.cross(m_collisionData.collider1Normal);
                impulsePerMove = m_rigidbodies[i]->inverseInertiaWorld * impulsePerMove;

                Vector3 angularInertiaWorld = impulsePerMove.cross(contactRelative);
                angularInertia[i] = angularInertiaWorld.dot(m_collisionData.collider1Normal);
                linearInertia[i] = m_rigidbodies[i]->getInverseMass();

                rotationPerMove[i] = impulsePerMove / angularInertia[i];

                totalInertia += linearInertia[i] + angularInertia[i];
            }
        }
        float inverseInertia = 1.0 / totalInertia;

        linearMove[0] = m_collisionData.depth * linearInertia[0] * inverseInertia;
        linearMove[1] = -m_collisionData.depth * linearInertia[1] * inverseInertia;
        angularMove[0] = m_collisionData.depth * angularInertia[0] * inverseInertia;
        angularMove[1] = -m_collisionData.depth * angularInertia[1] * inverseInertia;

        // Limit rotation
        float limitConstant = 0.2;
        for(int i = 0; i < 2; ++i) {
            if(m_rigidbodies[i] != nullptr) {
                Vector3 contactRelative = contactPoint - m_rigidbodies[i]->position;
                float limit = limitConstant * contactRelative.magnitude();

                float totalMove = angularMove[i] + linearMove[i];
                if(std::abs(angularMove[i]) > limit) {
                    angularMove[i] = angularMove[i] < 0.0 ? -limit : limit;
                }
                linearMove[i] = totalMove - angularMove[i];
            }
        }

        // Adjust positions and orientations
        m_rigidbodies[0]->position += m_collisionData.collider1Normal * linearMove[0];
        if(m_b2Dynamic) m_rigidbodies[1]->position += m_collisionData.collider1Normal * linearMove[1];

        m_rigidbodies[0]->orientation.addScaledVector(rotationPerMove[0] * angularMove[0], 1.0);
        if(m_b2Dynamic) m_rigidbodies[1]->orientation.addScaledVector(rotationPerMove[1] * angularMove[1], 1.0);
    }

    void CollisionResponse::applyImpulses() {
        Vector3 n = m_collisionData.collider1Normal;
        float elasticity = 0.3;

        Vector3 velocityChange1, velocityChange2, angularVelocityChange1, angularVelocityChange2;

        for(const Contact& contact : m_collisionData.contacts) {
            Vector3 r1 = contact.contactPoint - m_b1->position;
            Vector3 r2 = contact.contactPoint - m_b2->position;

            Vector3 closingVelocity = (m_rigidbodies[0]->linearVelocity + m_rigidbodies[0]->angularVelocity.cross(r1)) * -1.0;

            Vector3 velocityPerUnitImpulse1 = (m_rigidbodies[0]->inverseInertiaWorld * (r1.cross(n))).cross(r1);

            float deltaVelocity = velocityPerUnitImpulse1.dot(n);
            deltaVelocity += m_rigidbodies[0]->getInverseMass();

            if(m_b2Dynamic) {
                closingVelocity += m_rigidbodies[1]->linearVelocity + m_rigidbodies[1]->angularVelocity.cross(r2);

                Vector3 velocityPerUnitImpulse2 = (m_rigidbodies[1]->inverseInertiaWorld * (r2.cross(n))).cross(r2);

                deltaVelocity += velocityPerUnitImpulse2.dot(n);
                deltaVelocity += m_rigidbodies[1]->getInverseMass();
            }

            float cn = (-1 - elasticity) * closingVelocity.dot(n);
            Vector3 impulse = n * (cn / deltaVelocity);

            velocityChange1 -= impulse * m_rigidbodies[0]->getInverseMass();
            angularVelocityChange1 += m_rigidbodies[0]->inverseInertia * (impulse.cross(r1));

            if(m_b2Dynamic) {
                velocityChange2 += impulse * m_rigidbodies[1]->getInverseMass();
                angularVelocityChange2 -= m_rigidbodies[1]->inverseInertia * (impulse.cross(r2));
            }
        }

        m_rigidbodies[0]->linearVelocity += velocityChange1;
        m_rigidbodies[0]->angularVelocity += angularVelocityChange1;

        if(m_b2Dynamic) {
            m_rigidbodies[1]->linearVelocity += velocityChange2;
            m_rigidbodies[1]->angularVelocity += angularVelocityChange2;
        }
    }

}