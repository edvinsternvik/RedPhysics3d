#pragma once
#include "../Collision/CollisionData.h"

namespace redPhysics3d {

    class RigidBody;

    class CollisionResolver {
    public:
        CollisionResolver(CollisionData& collisionData);
        
        void solveCollision(unsigned int iterations);

    private:
        void resolveContact(Contact& contact, Vector3 linearMoveChange[2], Vector3 angularMoveChange[2]);
        void updatePenetrations(CollisionBody* body1, CollisionBody* body2, Vector3 linearMoveChange[2], Vector3 angularMoveChange[2]);
        void applyImpulses();

    private:
        CollisionData& m_collisionData;
    };

}