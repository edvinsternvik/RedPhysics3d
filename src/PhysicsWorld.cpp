#include "PhysicsWorld.h"
#include "Response/CollisionResponse.h"

#include "Math/Matrix3x3.h"
#include "Math/Quaternion.h"

namespace redPhysics3d {

    void PhysicsWorld::stepSimulation(float deltaTime) {
        for(auto& rb : m_rigidbodies) {
            rb->updateRotationMatricies();
            rb->updateInertia();

            rb->integrate(deltaTime);
        }

        for(auto& rb : m_rigidbodies) {
            for(auto& cs : rb->collisionShapes) {
                cs->updateCollisionShape();
            }
        }

        calculateRigidBodyCollisions();
        calculateStaticBodyCollisions();
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