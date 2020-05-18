#pragma once
#include "../Collision/CollisionData.h"

namespace redPhysics3d {

    class CollisionResponse {
    public:
        
        static void collisionResponse(const CollisionData& collisionData, CollisionBody* b1, CollisionBody* b2);

    };

}