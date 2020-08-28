#pragma once
#include "CollisionBody.h"

namespace redPhysics3d {

    class RigidBody : public CollisionBody {
    public:
        RigidBody();

        virtual CollisionBodyType getCollisionBodyType() const { return CollisionBodyType::Dynamic; }

        void integrate(float deltaTime);

        float getMass() const { return m_mass; };
        float getInverseMass() const { return m_inverseMass; }
        void setMass(const float& mass);

        void addForce(const Vector3& force);
        void addForceAtPoint(const Vector3& force, const Vector3& point);
        void addTorque(const Vector3& force);

        void clearForce();
        void clearTorque();
        void updateInertia();

    public:
        Vector3 linearVelocity, angularVelocity;
        Matrix3x3 inertia, inverseInertia;
        Matrix3x3 inverseInertiaWorld;
        float gravityScale = 1.0;
        bool lockRotation = false;
        Vector3 acceleration;

    private:
        Vector3 m_externalForce, m_externalTorque;
        float m_mass = 1.0, m_inverseMass = 1.0;

        float damping = 0.995;
        friend class PhysicsWorld;
    };

}