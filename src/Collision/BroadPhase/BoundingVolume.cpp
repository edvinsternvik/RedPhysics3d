#include "BoundingVolume.h"
#include "../../CollisionBody/CollisionBody.h"
#include "../../Math/Etc.h"

namespace redPhysics3d {

    BoundingVolume::BoundingVolume(const Vector3& position, const Vector3& halfSize) : position(position), halfSize(halfSize) {

    }

    BoundingVolume::BoundingVolume(CollisionBody* collisionBody) : position(collisionBody->position) {
        calculate(collisionBody);
    }

    void BoundingVolume::calculate(CollisionBody* collisionBody) {
        halfSize = Vector3(0.0, 0.0, 0.0);
        for(auto& shape : collisionBody->collisionShapes) {
            halfSize.x = std::max(halfSize.x, shape->AABBmax.x);
            halfSize.y = std::max(halfSize.y, shape->AABBmax.y);
            halfSize.z = std::max(halfSize.z, shape->AABBmax.z);
        }
    }

    bool BoundingVolume::intersects(const BoundingVolume& other) const {
        return (position.x + halfSize.x >= other.position.x - other.halfSize.x) && (position.x - halfSize.x <= other.position.x + other.halfSize.x)
            && (position.y + halfSize.y >= other.position.y - other.halfSize.y) && (position.y - halfSize.y <= other.position.y + other.halfSize.y)
            && (position.z + halfSize.z >= other.position.z - other.halfSize.z) && (position.z - halfSize.z <= other.position.z + other.halfSize.z);
    }

    float BoundingVolume::getSize() const {
        Vector3 size = halfSize + halfSize;
        return size.x * size.y * size.z;
    }

    float BoundingVolume::getGrowth(const BoundingVolume& otherVolume) const {
        Vector3 l = max(position + halfSize, otherVolume.position + otherVolume.halfSize);
        Vector3 s = min(position - halfSize, otherVolume.position - otherVolume.halfSize);

        Vector3 newHalfSize = (l - s) * 0.5;
        return newHalfSize.x + newHalfSize.y + newHalfSize.z - halfSize.x - halfSize.y - halfSize.z;
    }

}