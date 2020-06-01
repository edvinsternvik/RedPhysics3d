#pragma once
#include "../Collision/CollisionData.h"

namespace redPhysics3d {

    class RigidBody;

    class CollisionResponse {
    public:
        CollisionResponse(CollisionData collisionData, CollisionBody* b1, CollisionBody* b2);
        
        void solveCollision();

    private:
        void resolveContact(Contact& contact, Vector3 linearMoveChange[2], Vector3 angularMoveChange[2]);
        void updatePenetrations(Vector3 linearMoveChange[2], Vector3 angularMoveChange[2]);
        void applyImpulses();

    private:
        CollisionData m_collisionData;
        CollisionBody* m_b1, *m_b2;
        RigidBody* m_rigidbodies[2];
        bool m_b2Dynamic;
    };

}