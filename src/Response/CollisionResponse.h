#pragma once
#include "../Collision/CollisionData.h"

namespace redPhysics3d {

    class RigidBody;

    class CollisionResponse {
    public:
        CollisionResponse(const CollisionData& collisionData, CollisionBody* b1, CollisionBody* b2);
        
        void solveCollision();

    private:
        void resolveContact(const Vector3& contactPoint);
        void applyImpulses();

    private:
        const CollisionData& m_collisionData;
        CollisionBody* m_b1, *m_b2;
        RigidBody* m_rigidbodies[2];
        bool m_b2Dynamic;
    };

}