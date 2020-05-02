#include "CollisionShape.h"

namespace redPhysics3d {

    CollisionShape::CollisionShape() {

    }

    bool CollisionShape::testAABBCollision(const CollisionShape* const o) {
        return (getPosition().x + AABBmax.x >= o->getPosition().x + o->AABBmin.x && getPosition().x + AABBmin.x <= o->getPosition().x + o->AABBmax.x
             && getPosition().y + AABBmax.y >= o->getPosition().y + o->AABBmin.y && getPosition().y + AABBmin.y <= o->getPosition().y + o->AABBmax.y
             && getPosition().z + AABBmax.z >= o->getPosition().z + o->AABBmin.z && getPosition().z + AABBmin.z <= o->getPosition().z + o->AABBmax.z);
    }

}