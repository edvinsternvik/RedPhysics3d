#include "CollisionBody.h"

#define twopi 6.28318530718

namespace redPhysics3d {

    CollisionBody::CollisionBody() {
        
    }

    void CollisionBody::setPosition(const Vector3& position) {
        m_position = position;
    }

    void CollisionBody::setRotation(const Vector3& rotation) {
        if(m_rotation.x == rotation.x && m_rotation.y == rotation.y && m_rotation.z == rotation.z) return;
        m_rotation = rotation;

        if(m_rotation.x > twopi) m_rotation.x -= twopi;
        if(m_rotation.y > twopi) m_rotation.y -= twopi;
        if(m_rotation.z > twopi) m_rotation.z -= twopi;

        if(m_rotation.x < -twopi) m_rotation.x += twopi;
        if(m_rotation.y < -twopi) m_rotation.y += twopi;
        if(m_rotation.z < -twopi) m_rotation.z += twopi;

        for(auto& c : collisionShapes) c->updateCollisionShape();
    }


    bool CollisionBody::removeCollisionShape(CollisionShape* collisionShape) {
        for(int i = 0; i < collisionShapes.size(); ++i) {
            if(collisionShapes[i].get() == collisionShape) {
                collisionShapes.erase(collisionShapes.begin() + i);
                return true;
            }
        }

        return false;
    }

}