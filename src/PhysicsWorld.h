#pragma once
#include "Math/Vector.h"
#include "Collision/CollisionDispatcher.h"

#include <vector>
#include <memory>

namespace redPhysics3d {

    class PhysicsWorld {
    public:
        void stepSimulation(float deltaTime);

        template<class T>
        T* addCollisionShape() {
            std::unique_ptr<T> newCollisionShape = std::make_unique<T>();
            T* newCollisionShapePtr = newCollisionShape.get();
            m_collisionShapes.push_back(std::move(newCollisionShape));
            return newCollisionShapePtr;
        }

        bool removeCollisionShape(CollisionShape* collisionShape);
        
    public:
        Vector3 gravity;

    private:
        std::vector<std::unique_ptr<CollisionShape>> m_collisionShapes;
        CollisionDispatcher m_collisionDispatcher;
    };

}