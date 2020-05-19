#include "PhysicsWorld.h"
#include "Response/CollisionResponse.h"

namespace redPhysics3d {

    void PhysicsWorld::stepSimulation(float deltaTime) {
        calculateRigidBodyCollisions();
        calculateStaticBodyCollisions();

        for(auto& rb : m_rigidbodies) {
            Vector3 acceleration = rb->m_externalForce * rb->getInverseMass();

            rb->linearVelocity += acceleration * deltaTime;
            rb->setPosition(rb->getPosition() + rb->linearVelocity * deltaTime);

            rb->setRotation(rb->getRotation() + rb->angularVelocity * deltaTime);

            rb->clearForce();
        }
    }

    RigidBody* PhysicsWorld::addRigidBody() {
        m_rigidbodies.push_back(std::make_unique<RigidBody>());
        return m_rigidbodies.back().get();
    }

    StaticBody* PhysicsWorld::addStaticBody() {
        m_staticbodies.push_back(std::make_unique<StaticBody>());
        return m_staticbodies.back().get();
    }

    void PhysicsWorld::removeRigidBody(RigidBody* rigidbody) {
        for(int i = 0; i < m_rigidbodies.size(); ++i) {
            if(m_rigidbodies[i].get() == rigidbody) {
                m_rigidbodies.erase(m_rigidbodies.begin() + i);
                return;
            }
        }
    }

    void PhysicsWorld::removeStaticBody(StaticBody* staticbody) {
        for(int i = 0; i < m_staticbodies.size(); ++i) {
            if(m_staticbodies[i].get() == staticbody) {
                m_staticbodies.erase(m_staticbodies.begin() + i);
                return;
            }
        }
    }

    void PhysicsWorld::calculateRigidBodyCollisions() {
        for(int i = 0; i < m_rigidbodies.size(); ++i) {
            for(int j = i + 1; j < m_rigidbodies.size(); ++j) {
                for(auto& collisionShape1 : m_rigidbodies[i]->collisionShapes) {
                    for(auto& collisionShape2 : m_rigidbodies[j]->collisionShapes) {

                        if(collisionShape1->testAABBCollision(collisionShape2.get())) {
                            CollisionAlgorithm* collisionTestAlgorithm = m_collisionDispatcher.getCollisionAlgorithm(collisionShape1->getShapeType(), collisionShape2->getShapeType());

                            CollisionData collisionData = collisionTestAlgorithm->testCollision(collisionShape1.get(), collisionShape2.get());
                            if(collisionData.collided) {
                                CollisionResponse::collisionResponse(collisionData, m_rigidbodies[i].get(), m_rigidbodies[j].get());
                            }
                        }

                    }
                }
            }
        }
    }

    void PhysicsWorld::calculateStaticBodyCollisions() {
        for(int i = 0; i < m_rigidbodies.size(); ++i) {
            for(int j = 0; j < m_staticbodies.size(); ++j) {
                for(auto& collisionShape1 : m_rigidbodies[i]->collisionShapes) {
                    for(auto& collisionShape2 : m_staticbodies[j]->collisionShapes) {

                        if(collisionShape1->testAABBCollision(collisionShape2.get())) {
                            CollisionAlgorithm* collisionTestAlgorithm = m_collisionDispatcher.getCollisionAlgorithm(collisionShape1->getShapeType(), collisionShape2->getShapeType());

                            CollisionData collisionData = collisionTestAlgorithm->testCollision(collisionShape1.get(), collisionShape2.get());
                            if(collisionData.collided) {
                                CollisionResponse::collisionResponse(collisionData, m_rigidbodies[i].get(), m_staticbodies[j].get());
                            }
                        }

                    }
                }
            }
        }
    }



}