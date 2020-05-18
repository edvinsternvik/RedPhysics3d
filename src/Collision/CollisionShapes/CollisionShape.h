#pragma once
#include "../../Math/Vector.h"

namespace redPhysics3d {

    enum CollisionShapeType {
        Box = 0, 
        Count // To keep track of how many there are
    };

    class CollisionBody;

    class CollisionShape {
    public:
        CollisionShape(CollisionBody* collisionBody);
        CollisionShape(CollisionBody* collisionBody, const Vector3& position);
        CollisionShape(CollisionBody* collisionBody, const Vector3& position, const Vector3& rotation);
        CollisionShape(CollisionBody* collisionBody, const Vector3& position, const Vector3& rotation, const Vector3& size);
        virtual CollisionShapeType getShapeType() = 0;

        bool testAABBCollision(const CollisionShape* const o);
        virtual bool testCollision(const CollisionShape* const o) = 0;

        Vector3 getPosition() const;
        Vector3 getRotation() const;
        const Vector3& getSize() const { return m_size; }
        virtual void setPosition(const Vector3& newPosition) { m_position = newPosition; }
        virtual void setRotation(const Vector3& newRotation);
        virtual void setSize(const Vector3& newSize);

    private:
        virtual void updateCollisionShape();
        virtual void updateAABBsize() = 0;

    public:
        Vector3 AABBmin, AABBmax;
    
    protected:
        Vector3 m_position, m_rotation, m_size;

    private:
        CollisionBody* m_collisionBody;
    };

}