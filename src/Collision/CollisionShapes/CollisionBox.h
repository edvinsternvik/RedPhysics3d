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

        const Matrix3x3& getRotationMatrix() const;
        const Matrix3x3& getInvertedRotationMatrix() const;

    private:
        std::array<Vector3, 8> getBoxVerticies();

        void updateRotationMatricies();
        virtual void updateCollisionShape();

    public:
        std::array<Vector3, 8> verticies;

    private:
        Matrix3x3 m_rotationMatrix, m_invertedRotationMatrix;
    };

}