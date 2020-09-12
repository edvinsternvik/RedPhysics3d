#pragma once
#include "../Math/Vector.h"
#include "../Math/Quaternion.h"
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

        virtual CollisionBodyType getCollisionBodyType() const = 0;

        template<class T>
        T* addCollisionShape() {
            std::unique_ptr<T> newCollisionShape = std::make_unique<T>(this);
            T* newCollisionShapePtr = newCollisionShape.get();
            collisionShapes.push_back(std::move(newCollisionShape));
            return newCollisionShapePtr;
        }

        bool removeCollisionShape(CollisionShape* collisionShape);

        void updateRotationMatricies();

    public:
        std::vector<std::unique_ptr<CollisionShape>> collisionShapes;
        Quaternion orientation;
        Vector3 position;
        float bounciness = 0.2, friction = 0.5;

        Matrix3x3 rotationMatrix, invertedRotationMatrix;
    };

}