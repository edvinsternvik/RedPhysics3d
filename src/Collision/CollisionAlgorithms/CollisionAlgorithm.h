#pragma once
#include "../CollisionShapes/CollisionShape.h"
#include "../CollisionData.h"

namespace redPhysics3d {

    class CollisionAlgorithm {
    public:
        virtual CollisionData testCollision(CollisionShape* shape1, CollisionShape* shape2) = 0;
    };

    class BoxBoxCollisionAlgorithm : public CollisionAlgorithm {
        virtual CollisionData testCollision(CollisionShape* shape1, CollisionShape* shape2);
    };

}