#pragma once
#include "../CollisionShapes/CollisionShape.h"
#include "../CollisionData.h"

namespace redPhysics3d {

    class CollisionAlgorithm {
    public:
        virtual bool generateContacts(CollisionData& collisionData, CollisionShape* shape1, CollisionShape* shape2) = 0;
    };

    class BoxBoxCollisionAlgorithm : public CollisionAlgorithm {
        virtual bool generateContacts(CollisionData& collisionData, CollisionShape* shape1, CollisionShape* shape2);
    };

}