#pragma once
#include "CollisionShape.h"
#include <array>

namespace redPhysics3d {

    class CollisionBox : public CollisionShape {
    public:
        CollisionBox();

        virtual CollisionShapeType getShapeType() { return CollisionShapeType::Box; };

        virtual bool testCollision(const CollisionShape* const o) { return true; }
        virtual void updateAABBsize();

        const Vector3& getRotation() const { return m_rotation; }
        const Vector3& getSize() const { return m_size; }
        void setRotation(const Vector3& newRotation) { m_rotation = newRotation; }
        void setSize(const Vector3& newSize);

    private:
        std::array<Vector3, 8> getBoxVerticies();

    public:
        std::array<Vector3, 8> verticies;
        
    private:
        Vector3 m_rotation, m_size;
    };

}