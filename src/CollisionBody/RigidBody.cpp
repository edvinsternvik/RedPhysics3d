#include "RigidBody.h"

namespace redPhysics3d {

    RigidBody::RigidBody() {

    }

    void RigidBody::setMass(const float& mass) {
        if(mass <= 0.0) return;

        if(mass != m_mass) m_inverseMass = 1.0 / mass;
        m_mass = mass;
    }

    void RigidBody::addForce(const Vector3& force) {
        m_externalForce += force;
    }

    void RigidBody::addTorque(const Vector3& force) {
        m_externalTorque += force;
    }

    void RigidBody::clearForce() {
        m_externalForce = Vector3(0,0,0);
    }

    void RigidBody::clearTorque() {
        m_externalTorque = Vector3(0,0,0);
    }

}