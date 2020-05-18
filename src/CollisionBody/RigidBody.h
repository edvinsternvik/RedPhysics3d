#pragma once
#include "CollisionBody.h"

namespace redPhysics3d {

    class RigidBody : public CollisionBody {
    public:
        RigidBody();

    public:
        Vector3 velocity;
        float mass, elasticity;
        
    };

}