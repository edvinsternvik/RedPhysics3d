#include "CollisionShape.h"

namespace redPhysics3d {

    CollisionShape::CollisionShape() : m_size(1.0, 1.0, 1.0) {

    }

    CollisionShape::CollisionShape(const Vector3& position) : m_position(position), m_size(1.0, 1.0, 1.0) {

    }

    CollisionShape::CollisionShape(const Vector3& position, const Vector3& rotation) : m_position(position), m_rotation(rotation), m_size(1.0, 1.0, 1.0) {

    }
    
    CollisionShape::CollisionShape(const Vector3& position, const Vector3& rotation, const Vector3& size) : m_position(position), m_rotation(rotation), m_size(size) {

    }

    bool CollisionShape::testAABBCollision(const CollisionShape* const o) {
        return (getPosition().x + AABBmax.x >= o->getPosition().x + o->AABBmin.x && getPosition().x + AABBmin.x <= o->getPosition().x + o->AABBmax.x
             && getPosition().y + AABBmax.y >= o->getPosition().y + o->AABBmin.y && getPosition().y + AABBmin.y <= o->getPosition().y + o->AABBmax.y
             && getPosition().z + AABBmax.z >= o->getPosition().z + o->AABBmin.z && getPosition().z + AABBmin.z <= o->getPosition().z + o->AABBmax.z);
    }

    void CollisionShape::setRotation(const Vector3& newRotation) {
        m_rotation = newRotation;
        updateCollisionShape();
    }

    void CollisionShape::setSize(const Vector3& newSize) {
        m_size = newSize;
        updateCollisionShape();
    }

    void CollisionShape::updateCollisionShape() {
        updateAABBsize();
    }

}