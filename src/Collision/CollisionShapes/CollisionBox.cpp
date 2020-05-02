#include "CollisionBox.h"
#include <cmath>

namespace redPhysics3d {

    CollisionBox::CollisionBox() : CollisionShape() {
        updateCollisionShape();
    }

    void CollisionBox::updateAABBsize() {
        AABBmin = AABBmax = verticies[0];
        for(int i = 1; i < 8; ++i) {
            if(verticies[i].x < AABBmin.x) AABBmin.x = verticies[i].x;
            if(verticies[i].y < AABBmin.y) AABBmin.y = verticies[i].y;
            if(verticies[i].z < AABBmin.z) AABBmin.z = verticies[i].z;
            if(verticies[i].x > AABBmax.x) AABBmax.x = verticies[i].x;
            if(verticies[i].y > AABBmax.y) AABBmax.y = verticies[i].y;
            if(verticies[i].z > AABBmax.z) AABBmax.z = verticies[i].z;
        }
    }

    std::array<Vector3, 8> CollisionBox::getBoxVerticies() {
        Vector3 hs = m_size; // * 0.5
        return { Vector3(hs.x,  hs.y, hs.z) * m_rotationMatrix, Vector3(hs.x,  hs.y, -hs.z)  * m_rotationMatrix, Vector3(-hs.x,  hs.y, -hs.z) * m_rotationMatrix, Vector3(-hs.x,  hs.y, hs.z) * m_rotationMatrix,
                 Vector3(hs.x, -hs.y, hs.z) * m_rotationMatrix, Vector3(hs.x, -hs.y, -hs.z) * m_rotationMatrix, Vector3(-hs.x, -hs.y, -hs.z) * m_rotationMatrix, Vector3(-hs.x, -hs.y, hs.z) * m_rotationMatrix};
    }

    const Matrix3x3& CollisionBox::getRotationMatrix() const {
        return m_rotationMatrix;
    }
    
    const Matrix3x3& CollisionBox::getInvertedRotationMatrix() const {
        return m_invertedRotationMatrix;
    }

    void CollisionBox::updateRotationMatricies() {
        m_rotationMatrix  = Matrix3x3::getRotationMatrixX(getRotation().x);
        m_rotationMatrix *= Matrix3x3::getRotationMatrixY(getRotation().y);
        m_rotationMatrix *= Matrix3x3::getRotationMatrixZ(getRotation().z);

        // m_invertedRotationMatrix = m_rotationMatrix.inverse();
        m_invertedRotationMatrix = Matrix3x3::getRotationMatrixZ(-getRotation().z);
        m_invertedRotationMatrix *= Matrix3x3::getRotationMatrixY(-getRotation().y);
        m_invertedRotationMatrix *= Matrix3x3::getRotationMatrixX(-getRotation().x);
    }

    void CollisionBox::updateCollisionShape() {
        updateRotationMatricies();
        verticies = getBoxVerticies();
        updateAABBsize();
    }
    
}