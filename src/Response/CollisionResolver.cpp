#include "CollisionResolver.h"

#include "../CollisionBody/RigidBody.h"

#include "../PhysicsWorld.h"

namespace redPhysics3d {

    CollisionResolver::CollisionResolver(CollisionData& collisionData) : m_collisionData(collisionData) {
    }

    void CollisionResolver::solveCollision(unsigned int iterations, float deltaTime) {
        // Resolve interpenetration using nonlinear projection
        for(int i = 0; i < iterations; ++i) {
            Contact* worstContact = nullptr;
            for(Contact& contact : m_collisionData.contacts) {
                if(worstContact == nullptr || (contact.penetration > worstContact->penetration && contact.penetration > 0.0)) worstContact = &contact;
            }
            if(worstContact == nullptr || worstContact->penetration < 0.0) break;

            Vector3 linearMove[2] = {Vector3(), Vector3()}, angularMove[2] = {Vector3(), Vector3()};
            resolveContact(*worstContact, linearMove, angularMove);
            updatePenetrations(worstContact->colliders[0]->getCollisionBody(), worstContact->colliders[1]->getCollisionBody(), linearMove, angularMove);
        }

        // Change velocities
        applyImpulses(deltaTime);
    }

    void CollisionResolver::resolveContact(Contact& contact, Vector3 linearMoveChange[2], Vector3 angularMoveChange[2]) {
        RigidBody* rigidbodies[2];
        rigidbodies[0] = contact.colliders[0]->getCollisionBody()->getCollisionBodyType() == CollisionBodyType::Dynamic ? (RigidBody*)contact.colliders[0]->getCollisionBody() : nullptr;
        rigidbodies[1] = contact.colliders[1]->getCollisionBody()->getCollisionBodyType() == CollisionBodyType::Dynamic ? (RigidBody*)contact.colliders[1]->getCollisionBody() : nullptr;

        float linearMove[2] = {0.0, 0.0}, angularMove[2] = {0.0, 0.0};
        float totalInertia = 0.0, linearInertia[2] = {0.0, 0.0}, angularInertia[2] = {0.0,0.0};
        Vector3 rotationPerMove[2] = { Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0) };
        for(int i = 0; i < 2; ++i) {
            if(rigidbodies[i] != nullptr) {
                Vector3 contactRelative = contact.contactPoint - rigidbodies[i]->position;

                Vector3 impulsePerMove = contactRelative.cross(contact.collider1Normal);
                impulsePerMove = rigidbodies[i]->inverseInertiaWorld * impulsePerMove;

                Vector3 angularInertiaWorld = impulsePerMove.cross(contactRelative);
                angularInertia[i] = angularInertiaWorld.dot(contact.collider1Normal);
                linearInertia[i] = rigidbodies[i]->getInverseMass();

                rotationPerMove[i] = impulsePerMove / angularInertia[i];

                totalInertia += linearInertia[i] + angularInertia[i];
            }
        }
        float inverseInertia = 1.0 / totalInertia;

        linearMove[0] = contact.penetration * linearInertia[0] * inverseInertia;
        linearMove[1] = -contact.penetration * linearInertia[1] * inverseInertia;
        angularMove[0] = contact.penetration * angularInertia[0] * inverseInertia;
        angularMove[1] = -contact.penetration * angularInertia[1] * inverseInertia;

        // Limit or lock rotation
        float limitConstant = 0.001;
        for(int i = 0; i < 2; ++i) {
            if(rigidbodies[i] != nullptr) {
                float totalMove = angularMove[i] + linearMove[i];
                if(rigidbodies[i]->lockRotation) {
                    angularMove[i] = 0.0;
                }
                else {
                    Vector3 contactRelative = contact.contactPoint - rigidbodies[i]->position;
                    float limit = limitConstant * contactRelative.magnitude();

                    if(std::abs(angularMove[i]) > limit) {
                        angularMove[i] = angularMove[i] < 0.0 ? -limit : limit;
                    }
                }
                linearMove[i] = totalMove - angularMove[i];
            }
        }

        // Adjust positions and orientations
        if(rigidbodies[0]) {
            linearMoveChange[0] = contact.collider1Normal * linearMove[0];
            rigidbodies[0]->position += linearMoveChange[0];

            angularMoveChange[0] = rotationPerMove[0] * angularMove[0];
            rigidbodies[0]->orientation.addScaledVector(angularMoveChange[0], 1.0);
        }

        if(rigidbodies[1]) {
            linearMoveChange[1] = contact.collider1Normal * linearMove[1];
            rigidbodies[1]->position += linearMoveChange[1];

            angularMoveChange[1] = rotationPerMove[1] * angularMove[1];
            rigidbodies[1]->orientation.addScaledVector(angularMoveChange[1], 1.0);
        }
    }


    void CollisionResolver::updatePenetrations(CollisionBody* body1, CollisionBody* body2, Vector3 linearMoveChange[2], Vector3 angularMoveChange[2]) {
        for(Contact& contact : m_collisionData.contacts) {
            for(int i = 0; i < 2; ++i) {
                int index = -1;
                if(contact.colliders[i]->getCollisionBody() == body1) index = 0;
                else if(contact.colliders[i]->getCollisionBody() == body2) index = 1;

                if(index > -1) {
                    Vector3 contactRelative = contact.contactPoint - contact.colliders[i]->getCollisionBody()->position;
                    Vector3 deltaPosition = angularMoveChange[index].cross(contactRelative) + linearMoveChange[index];
                    contact.penetration += deltaPosition.dot(contact.collider1Normal) * (i == 1 ? 1.0 : -1.0);
                }
            }
        }
    }

    void CollisionResolver::applyImpulses(float deltaTime) {
        for(Contact& contact : m_collisionData.contacts) {
            contact.prepareContact();
            
            RigidBody* rigidbodies[2];
            getRigidBodies(&contact, rigidbodies);

            Matrix3x3 velPerImpulse = getVelocityPerUnitImpulse(&contact);
            Matrix3x3 impulsePerVel = velPerImpulse.inverse();

            Vector3 closingVelocity = getClosingVelocity(&contact);
            Vector3 desiredDeltaVelocity = getDesiredDeltaVelocity(&contact, closingVelocity, deltaTime);

            Vector3 impulseContact = impulsePerVel * desiredDeltaVelocity;

            float friction = 0.5;
            float planarImpulse = std::sqrt(impulseContact.y * impulseContact.y + impulseContact.z * impulseContact.z);
            if(planarImpulse > -impulseContact.x * friction) {
                float temp = impulseContact.y;
                impulseContact.y /= planarImpulse;
                impulseContact.z /= planarImpulse;

                impulseContact.x = -(velPerImpulse.at(0, 0) +
                    velPerImpulse.at(1,0) * friction * impulseContact.y +
                    velPerImpulse.at(2,0) * friction * impulseContact.z);
                impulseContact.x = -desiredDeltaVelocity.x / impulseContact.x;
                impulseContact.y *= std::abs(impulseContact.x) * friction;
                impulseContact.z *= std::abs(impulseContact.x) * friction;
            }

            Vector3 impulse = contact.worldToContactMatrix.transpose() * impulseContact;

            if(rigidbodies[0]) applyImpulseToRigidbody(rigidbodies[0], impulse * -1.0, contact.relativeContactPosition[0]);
            if(rigidbodies[1]) applyImpulseToRigidbody(rigidbodies[1], impulse       , contact.relativeContactPosition[1]);
        }

    }

    Matrix3x3 CollisionResolver::getVelocityPerUnitImpulse(const Contact* contact) {
        RigidBody* rigidbodies[2];
        getRigidBodies(contact, rigidbodies);

        Matrix3x3 deltaVelWorld = Matrix3x3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        float inverseMass = 0.0;

        // Velocity from angular motion
        for(int i = 0; i < 2; ++i) {
            if(rigidbodies[i]) {
                Matrix3x3 impulseToTorque = Matrix3x3::getSkewSymmetric(contact->relativeContactPosition[i]);
                
                Matrix3x3 dVelWorld = impulseToTorque;
                dVelWorld = dVelWorld * rigidbodies[i]->inverseInertiaWorld;
                dVelWorld = dVelWorld * impulseToTorque;
                dVelWorld *= -1.0f;

                deltaVelWorld += dVelWorld;

                inverseMass += rigidbodies[i]->getInverseMass();
            }
        }

        Matrix3x3 deltaVelocity = contact->worldToContactMatrix;
        deltaVelocity = deltaVelocity * deltaVelWorld;
        deltaVelocity = deltaVelocity * contact->worldToContactMatrix.transpose();

        // Velocity from linear motion
        for(int i = 0; i < 3; ++i) deltaVelocity.at(i, i) += inverseMass;

        return deltaVelocity;
    }

    Vector3 CollisionResolver::getClosingVelocity(const Contact* contact) {
        RigidBody* rigidbodies[2];
        getRigidBodies(contact, rigidbodies);

        Vector3 contactVelocity = Vector3(0.0, 0.0, 0.0);

        for(int i = 0; i < 2; ++i) {
            float direction = (i == 0) ? -1.0 : 1.0;

            if(rigidbodies[i]) {
                Vector3 velocity = rigidbodies[i]->angularVelocity.cross(contact->relativeContactPosition[i]);
                velocity += rigidbodies[i]->linearVelocity;
                velocity = velocity * direction;
                contactVelocity += contact->worldToContactMatrix * velocity;
            }
        }

        return contactVelocity;
    }

    Vector3 CollisionResolver::getDesiredDeltaVelocity(Contact* contact, const Vector3& closingVelocity, const float& deltaTime) {
        RigidBody* rigidbodies[2];
        getRigidBodies(contact, rigidbodies);
        
        float coefficientOfRestitution = (contact->colliders[0]->getCollisionBody()->bounciness + contact->colliders[1]->getCollisionBody()->bounciness) * 0.5;
        float deltaVelFromAcceleration = 0.0;
        if(rigidbodies[0]) deltaVelFromAcceleration += rigidbodies[0]->acceleration.dot(contact->collider1Normal) * deltaTime;
        if(rigidbodies[1]) deltaVelFromAcceleration -= rigidbodies[1]->acceleration.dot(contact->collider1Normal) * deltaTime;

        if(std::abs(closingVelocity.x) < 0.1) coefficientOfRestitution = 0.0;

        float desiredDeltaVelocity = -closingVelocity.x - coefficientOfRestitution * (closingVelocity.x - deltaVelFromAcceleration);

        return Vector3(desiredDeltaVelocity, -closingVelocity.y, -closingVelocity.z);
    }

    void CollisionResolver::applyImpulseToRigidbody(RigidBody* rigidBody, const Vector3& impulse, const Vector3& contactPosition) {
        Vector3 velocityChange = impulse * rigidBody->getInverseMass();

        Vector3 rotationChange = rigidBody->inverseInertia * (impulse.cross(contactPosition)) * -1.0;

        rigidBody->linearVelocity += velocityChange;
        rigidBody->angularVelocity += rotationChange;
    }

    void CollisionResolver::getRigidBodies(const Contact* contact, RigidBody* rb[2]) {
        if(contact) {
            rb[0] = contact->colliders[0]->getCollisionBody()->getCollisionBodyType() == CollisionBodyType::Dynamic ? (RigidBody*)contact->colliders[0]->getCollisionBody() : nullptr;
            rb[1] = contact->colliders[1]->getCollisionBody()->getCollisionBodyType() == CollisionBodyType::Dynamic ? (RigidBody*)contact->colliders[1]->getCollisionBody() : nullptr;
        }
        else {
            rb[0] = rb[1] = nullptr;
        }
    }

}