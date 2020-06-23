#include "CollisionShape.h"
#include "../../CollisionBody/CollisionBody.h"

namespace redPhysics3d {

    CollisionShape::CollisionShape(CollisionBody* collisionBody) : m_collisionBody(collisionBody), size(1.0, 1.0, 1.0) {

    }

    CollisionShape::CollisionShape(CollisionBody* collisionBody, const Vector3& position) : m_collisionBody(collisionBody), size(1.0, 1.0, 1.0) {

    }

    CollisionShape::CollisionShape(CollisionBody* collisionBody, const Vector3& position, const Vector3& rotation) : m_collisionBody(collisionBody), size(1.0, 1.0, 1.0) {

    }
    
    CollisionShape::CollisionShape(CollisionBody* collisionBody, const Vector3& position, const Vector3& rotation, const Vector3& size) : m_collisionBody(collisionBody), size(size) {

    }

    bool CollisionShape::testAABBCollision(const CollisionShape* const o) {
        return (getPosition().x + AABBmax.x >= o->getPosition().x + o->AABBmin.x && getPosition().x + AABBmin.x <= o->getPosition().x + o->AABBmax.x
             && getPosition().y + AABBmax.y >= o->getPosition().y + o->AABBmin.y && getPosition().y + AABBmin.y <= o->getPosition().y + o->AABBmax.y
             && getPosition().z + AABBmax.z >= o->getPosition().z + o->AABBmin.z && getPosition().z + AABBmin.z <= o->getPosition().z + o->AABBmax.z);
    }

    void CollisionShape::updateCollisionShape() {
        updateAABBsize();
    }

    Vector3 CollisionShape::getPosition() const {
        return m_collisionBody->position + localPosition;
        }
    Quaternion CollisionShape::getOrientation() const {
        return m_collisionBody->orientation * localOrientation;
        }
    Matrix3x3 CollisionShape::getRotationMatrix() const {
        return getOrientation().calculateRotationMatrix();
    }
    Matrix3x3 CollisionShape::getInvertedRotationMatrix() const {
        return getOrientation().calculateRotationMatrix().transpose();
    }

}