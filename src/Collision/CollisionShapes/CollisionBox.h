#pragma once
#include "CollisionShape.h"
#include <array>

namespace redPhysics3d {

    class CollisionBox : public CollisionShape {
    public:
        CollisionBox(CollisionBody* collisionBody);

        virtual CollisionShapeType getShapeType() { return CollisionShapeType::Box; };

        virtual bool testCollision(const CollisionShape* const o) { return true; }
        virtual void updateAABBsize();

        std::array<Vector3, 4> getFaceVerticies(int index);

    private:
        std::array<Vector3, 8> getBoxVerticies();

        virtual void updateCollisionShape();

    public:
        std::array<Vector3, 8> verticies;
    };

}