#pragma once
#include "CollisionBody.h"

namespace redPhysics3d {

    class RigidBody : public CollisionBody {
    public:
        RigidBody();

        virtual CollisionBodyType getCollisionBodyType() { return CollisionBodyType::Dynamic; }

        float getMass() const { return m_mass; };
        Vector3 getInertia() const { return m_inertia; }
        float getInverseMass() const { return m_inverseMass; }
        Vector3 getInverseInertia() const { return m_inverseInertia; }
        void setMass(const float& mass);

        void addForce(const Vector3& force);
        void addTorque(const Vector3& force);

        void clearForce();
        void clearTorque();

    private:
        void calculateInertia();

    public:
        Vector3 linearVelocity, angularVelocity;

    private:
        Vector3 m_externalForce, m_externalTorque;
        float m_mass = 1.0, m_elasticity, m_inverseMass = 1.0;
        Vector3 m_inertia, m_inverseInertia;

        friend class PhysicsWorld;
    };

}