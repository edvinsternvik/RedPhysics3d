#include "PhysicsWorld.h"
#include "Response/CollisionResponse.h"


namespace redPhysics3d {

    void PhysicsWorld::stepSimulation(float deltaTime) {
        for(int i = 0; i < m_collisionShapes.size(); ++i) {
            for(int j = i + 1; j < m_collisionShapes.size(); ++j) {
                if(m_collisionShapes[i]->testAABBCollision(m_collisionShapes[j].get())) {
                    CollisionAlgorithm* collisionTestAlgorithm = m_collisionDispatcher.getCollisionAlgorithm(m_collisionShapes[i]->getShapeType(), m_collisionShapes[j]->getShapeType());

                    CollisionData collisionData = collisionTestAlgorithm->testCollision(m_collisionShapes[i].get(), m_collisionShapes[j].get());
                    if(collisionData.collided) {
                        CollisionResponse::collisionResponse(collisionData);
                    }
                }
            }
        }
    }

    bool PhysicsWorld::removeCollisionShape(CollisionShape* collisionShape) {
        for(int i = 0; i < m_collisionShapes.size(); ++i) {
            if(m_collisionShapes[i].get() == collisionShape) {
                m_collisionShapes.erase(m_collisionShapes.begin() + i);
                return true;
            }
        }

        return false;
    }

}