#pragma once
#include "CollisionBody.h"

namespace redPhysics3d {

    class StaticBody : public CollisionBody {
    public:
        StaticBody();

        virtual CollisionBodyType getCollisionBodyType() { return CollisionBodyType::Static; }

    public:
        
    };

}