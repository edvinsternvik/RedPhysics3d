#pragma once
#include "Math/Vector.h"
#include "Collision/CollisionDispatcher.h"
#include "CollisionBody/RigidBody.h"
#include "CollisionBody/StaticBody.h"

#include <vector>
#include <memory>

namespace redPhysics3d {

    class CollisionData;

    class PhysicsWorld {
    public:
        void stepSimulation(float deltaTime);

        RigidBody* addRigidBody();
        StaticBody* addStaticBody();
        void removeRigidBody(RigidBody* rigidbody);
        void removeStaticBody(StaticBody* staticbody);

    private:
        void generateRigidBodyContacts(CollisionData& collisionData);
        void generateStaticBodyContacts(CollisionData& collisionData);
        
    public:
        Vector3 gravity;

    private:
        std::vector<std::unique_ptr<RigidBody>> m_rigidbodies;
        std::vector<std::unique_ptr<StaticBody>> m_staticbodies;
        CollisionDispatcher m_collisionDispatcher;
    };

}