#pragma once
#include "../../Math/Vector.h"

namespace redPhysics3d {

    class CollisionBody;

    class BoundingVolume {
    public:
        BoundingVolume(const Vector3& position, const Vector3& halfSize);
        BoundingVolume(CollisionBody* collisionBody);

        void calculate(CollisionBody* collisionBody);
        bool intersects(const BoundingVolume& other) const;
        float getSize() const;
        float getGrowth(const BoundingVolume& otherVolume) const;

    public:
        Vector3 position, halfSize;
    };

}