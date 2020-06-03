#pragma once
#include "../../Math/Vector.h"
#include "../../Math/Quaternion.h"

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
        Quaternion getOrientation() const;
        Matrix3x3& getRotationMatrix();
        Matrix3x3& getInvertedRotationMatrix();
        CollisionBody* getCollisionBody() const { return m_collisionBody; }
        virtual void updateCollisionShape();

    protected:
        virtual void updateAABBsize() = 0;

        friend class CollisionBody;

    public:
        Vector3 AABBmin, AABBmax;
        Vector3 size;

    private:
        CollisionBody* m_collisionBody;
    };

}