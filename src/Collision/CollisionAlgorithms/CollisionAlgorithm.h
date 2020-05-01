#pragma once
#include "../CollisionShapes/CollisionShape.h"

namespace redPhysics3d {

    class CollisionAlgorithm {
    public:
        virtual bool testCollision(CollisionShape* shape1, CollisionShape* shape2) = 0;
    };

    class BoxBoxCollisionAlgorithm : public CollisionAlgorithm {
        virtual bool testCollision(CollisionShape* shape1, CollisionShape* shape2);
    };

}