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

        const Vector3& getPosition() const { return m_position; }
        const Vector3& getRotation() const { return m_rotation; }
        void setPosition(const Vector3& position);
        void setRotation(const Vector3& rotation);

        template<class T>
        T* addCollisionShape() {
            std::unique_ptr<T> newCollisionShape = std::make_unique<T>(this);
            T* newCollisionShapePtr = newCollisionShape.get();
            collisionShapes.push_back(std::move(newCollisionShape));
            return newCollisionShapePtr;
        }

        bool removeCollisionShape(CollisionShape* collisionShape);

    public:
        std::vector<std::unique_ptr<CollisionShape>> collisionShapes;

    private:
        Vector3 m_position, m_rotation;
    };

}