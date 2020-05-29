#include "RigidBody.h"

#include <cmath>

namespace redPhysics3d {

    RigidBody::RigidBody() {
        setMass(1.0);
    }

    void RigidBody::integrate(float deltaTime) {
        Vector3 acceleration = m_externalForce * getInverseMass();

        Vector3 angularAcceleration = inverseInertiaWorld * m_externalTorque;

        float dampingFactor = pow(damping, deltaTime);
        linearVelocity = linearVelocity * dampingFactor;
        angularVelocity = angularVelocity * dampingFactor;


        linearVelocity += acceleration * deltaTime;
        angularVelocity += angularAcceleration * deltaTime;

        position += linearVelocity * deltaTime;
        orientation.addScaledVector(angularVelocity, deltaTime);
        orientation.normalize();

        clearForce();
        clearTorque();
    }

    void RigidBody::setMass(const float& mass) {
        if(mass <= 0.0) return;

        if(mass != m_mass) m_inverseMass = 1.0 / mass;
        m_mass = mass;
    }

    void RigidBody::addForce(const Vector3& force) {
        m_externalForce += force;
    }

    void RigidBody::addForceAtPoint(const Vector3& force, const Vector3& point) {
        Vector3 p = point - position;
        m_externalForce += force;
        m_externalTorque += p.cross(force);
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

    void RigidBody::updateInertia() {
        if(collisionShapes.size() < 1) {
            return;
        }

        // 0,0833333333333 = 1 / 12
        float widthSquare = std::pow(collisionShapes[0]->size.x * 2.0, 2), heightSqure = std::pow(collisionShapes[0]->size.y * 2.0, 2), lengthSquare = std::pow(collisionShapes[0]->size.z * 2.0, 2);
        Vector3 temp = Vector3(0.0833333333333 * m_mass * (heightSqure + lengthSquare), 0.0833333333333 * m_mass * (lengthSquare + widthSquare), 0.0833333333333 * m_mass * (heightSqure + widthSquare));

        inertia = Matrix3x3(0,0,0,0,0,0,0,0,0);
        inertia.at(0,0) = temp.x;
        inertia.at(1,1) = temp.y;
        inertia.at(2,2) = temp.z;

        inverseInertia = inertia.inverse();

        inverseInertiaWorld = rotationMatrix * inverseInertia * invertedRotationMatrix;
    }

}