#include "CollisionResponse.h"

#include "../CollisionBody/RigidBody.h"

#include "../PhysicsWorld.h"

namespace redPhysics3d {

    CollisionResponse::CollisionResponse(CollisionData collisionData, CollisionBody* b1, CollisionBody* b2) : m_collisionData(collisionData), m_b1(b1), m_b2(b2) {
        m_b2Dynamic = m_b2->getCollisionBodyType() == CollisionBodyType::Dynamic;
        m_rigidbodies[0] = (RigidBody*)m_b1;
        m_rigidbodies[1] = m_b2Dynamic ? (RigidBody*)m_b2 : nullptr;
    }

    void CollisionResponse::solveCollision() {
        // Resolve interpenetration using nonlinear projection
        for(int i = 0; i < 5; ++i) {
            Contact* worstContact = nullptr;
            for(Contact& contact : m_collisionData.contacts) {
                if(worstContact == nullptr || contact.penetration > worstContact->penetration && contact.penetration > 0.0) worstContact = &contact;
            }
            if(worstContact == nullptr) break;

            Vector3 linearMove[2], angularMove[2];
            resolveContact(*worstContact, linearMove, angularMove);
            updatePenetrations(linearMove, angularMove);
        }

        // Change velocities
        applyImpulses();
    }

    void CollisionResponse::resolveContact(Contact& contact, Vector3 linearMoveChange[2], Vector3 angularMoveChange[2]) {
        float linearMove[2] = {0.0, 0.0}, angularMove[2] = {0.0, 0.0};
        float totalInertia = 0.0, linearInertia[2] = {0.0, 0.0}, angularInertia[2] = {0.0,0.0};
        Vector3 rotationPerMove[2];
        for(int i = 0; i < 2; ++i) {
            if(m_rigidbodies[i] != nullptr) {
                Vector3 contactRelative = contact.contactPoint - m_rigidbodies[i]->position;

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

        linearMove[0] = contact.penetration * linearInertia[0] * inverseInertia;
        linearMove[1] = -contact.penetration * linearInertia[1] * inverseInertia;
        angularMove[0] = contact.penetration * angularInertia[0] * inverseInertia;
        angularMove[1] = -contact.penetration * angularInertia[1] * inverseInertia;

        // Limit rotation
        float limitConstant = 0.2;
        for(int i = 0; i < 2; ++i) {
            if(m_rigidbodies[i] != nullptr) {
                Vector3 contactRelative = contact.contactPoint - m_rigidbodies[i]->position;
                float limit = limitConstant * contactRelative.magnitude();

                float totalMove = angularMove[i] + linearMove[i];
                if(std::abs(angularMove[i]) > limit) {
                    angularMove[i] = angularMove[i] < 0.0 ? -limit : limit;
                }
                linearMove[i] = totalMove - angularMove[i];
            }
        }

        // Adjust positions and orientations
        linearMoveChange[0] = m_collisionData.collider1Normal * linearMove[0];
        m_rigidbodies[0]->position += linearMoveChange[0];
        if(m_b2Dynamic) {
            linearMoveChange[1] = m_collisionData.collider1Normal * linearMove[1];
            m_rigidbodies[1]->position += linearMoveChange[1];
        }

        angularMoveChange[0] = rotationPerMove[0] * angularMove[0];
        m_rigidbodies[0]->orientation.addScaledVector(angularMoveChange[0], 1.0);
        if(m_b2Dynamic) {
            angularMoveChange[1] = rotationPerMove[1] * angularMove[1];
            m_rigidbodies[1]->orientation.addScaledVector(angularMoveChange[1], 1.0);
        }
    }

    void CollisionResponse::updatePenetrations(Vector3 linearMoveChange[2], Vector3 angularMoveChange[2]) {
        for(Contact& contact : m_collisionData.contacts) {
            for(int i = 0; i < 2; ++i) {
                if(m_rigidbodies[i]) {
                    Vector3 contactRelative = contact.contactPoint - m_rigidbodies[i]->position;
                    Vector3 deltaPosition = angularMoveChange[i].cross(contactRelative) + linearMoveChange[i];
                    contact.penetration += deltaPosition.dot(m_collisionData.collider1Normal) * (i == 1 ? 1.0 : -1.0);
                }
            }
        }
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