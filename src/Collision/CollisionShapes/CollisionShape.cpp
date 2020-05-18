#include "CollisionShape.h"
#include "../../CollisionBody/CollisionBody.h"

namespace redPhysics3d {

    CollisionShape::CollisionShape(CollisionBody* collisionBody) : m_collisionBody(collisionBody), m_size(1.0, 1.0, 1.0) {

    }

    CollisionShape::CollisionShape(CollisionBody* collisionBody, const Vector3& position) : m_collisionBody(collisionBody), m_position(position), m_size(1.0, 1.0, 1.0) {

    }

    CollisionShape::CollisionShape(CollisionBody* collisionBody, const Vector3& position, const Vector3& rotation) : m_collisionBody(collisionBody), m_position(position), m_rotation(rotation), m_size(1.0, 1.0, 1.0) {

    }
    
    CollisionShape::CollisionShape(CollisionBody* collisionBody, const Vector3& position, const Vector3& rotation, const Vector3& size) : m_collisionBody(collisionBody), m_position(position), m_rotation(rotation), m_size(size) {

    }

    bool CollisionShape::testAABBCollision(const CollisionShape* const o) {
        return (getPosition().x + AABBmax.x >= o->getPosition().x + o->AABBmin.x && getPosition().x + AABBmin.x <= o->getPosition().x + o->AABBmax.x
             && getPosition().y + AABBmax.y >= o->getPosition().y + o->AABBmin.y && getPosition().y + AABBmin.y <= o->getPosition().y + o->AABBmax.y
             && getPosition().z + AABBmax.z >= o->getPosition().z + o->AABBmin.z && getPosition().z + AABBmin.z <= o->getPosition().z + o->AABBmax.z);
    }

    Vector3 CollisionShape::getPosition() const {
        return m_collisionBody->position + m_position;
    }
    Vector3 CollisionShape::getRotation() const {
        return m_collisionBody->rotation + m_rotation;
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