#pragma once
#include "../Collision/CollisionData.h"

namespace redPhysics3d {

    class RigidBody;

    class CollisionResolver {
    public:
        CollisionResolver(CollisionData& collisionData);
        
        void solveCollision(unsigned int iterations, float deltaTime);

    private:
        void resolveContact(Contact& contact, Vector3 linearMoveChange[2], Vector3 angularMoveChange[2]);
        void updatePenetrations(CollisionBody* body1, CollisionBody* body2, Vector3 linearMoveChange[2], Vector3 angularMoveChange[2]);
        void applyImpulses(float deltaTime);
        Matrix3x3 getVelocityPerUnitImpulse(const Contact* contact);
        Vector3 getClosingVelocity(const Contact* contact);
        Vector3 getDesiredDeltaVelocity(Contact* contact, const Vector3& closingVelocity, const float& deltaTime);
        void applyImpulseToRigidbody(RigidBody* rigidBody, const Vector3& impulse, const Vector3& contactPosition);
        void getRigidBodies(const Contact* contact, RigidBody* rb[2]);

    private:
        CollisionData& m_collisionData;
    };

}