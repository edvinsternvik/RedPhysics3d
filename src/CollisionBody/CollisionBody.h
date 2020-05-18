#pragma once
#include "../Math/Vector.h"
#include "../Collision/CollisionShapes/CollisionShape.h"

#include <memory>
#include <vector>

namespace redPhysics3d {

    enum CollisionBodyType {
        Dynamic, Static
    };

    class CollisionBody {
    public:
        CollisionBody();

        virtual CollisionBodyType getCollisionBodyType() = 0;

        template<class T>
        T* addCollisionShape() {
            std::unique_ptr<T> newCollisionShape = std::make_unique<T>(this);
            T* newCollisionShapePtr = newCollisionShape.get();
            collisionShapes.push_back(std::move(newCollisionShape));
            return newCollisionShapePtr;
        }

        bool removeCollisionShape(CollisionShape* collisionShape);

    public:
        Vector3 position, rotation;
        std::vector<std::unique_ptr<CollisionShape>> collisionShapes;

    };

}