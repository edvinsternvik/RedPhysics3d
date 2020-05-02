#pragma once
#include "../../Math/Vector.h"

namespace redPhysics3d {

    enum CollisionShapeType {
        Box = 0, 
        Count // To keep track of how many there are
    };

    class CollisionShape {
    public:
        CollisionShape();
        virtual CollisionShapeType getShapeType() = 0;

        bool testAABBCollision(const CollisionShape* const o);
        virtual bool testCollision(const CollisionShape* const o) = 0;

        const Vector3& getPosition() const { return m_position; }
        void setPosition(const Vector3& newPosition) { m_position = newPosition; }

    private:
        virtual void updateAABBsize() = 0;

    public:
        Vector3 AABBmin, AABBmax;
    
    private:
        Vector3 m_position;
    };

}