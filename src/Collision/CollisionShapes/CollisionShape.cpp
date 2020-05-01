#include "CollisionShape.h"

namespace redPhysics3d {

    CollisionShape::CollisionShape() {

    }

    bool CollisionShape::testAABBCollision(const CollisionShape* const o) {
        return (getPosition().x + AABBsize >= o->getPosition().x - o->AABBsize && getPosition().x - AABBsize <= o->getPosition().x + o->AABBsize
             && getPosition().y + AABBsize >= o->getPosition().y - o->AABBsize && getPosition().y - AABBsize <= o->getPosition().y + o->AABBsize
             && getPosition().z + AABBsize >= o->getPosition().z - o->AABBsize && getPosition().z - AABBsize <= o->getPosition().z + o->AABBsize);
    }

}