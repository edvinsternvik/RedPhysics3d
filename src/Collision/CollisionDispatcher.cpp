#include "CollisionDispatcher.h"

namespace redPhysics3d {

    CollisionDispatcher::CollisionDispatcher() {
        initAlgorithmMatrix();
    }

    CollisionAlgorithm* CollisionDispatcher::getCollisionAlgorithm(CollisionShapeType shapeType1, CollisionShapeType shapeType2) {
        return m_algorithms[(int)shapeType1][(int)shapeType2].get();
    }

    void CollisionDispatcher::initAlgorithmMatrix() {
        for(int i = 0; i < CollisionShapeType::Count; ++i) {
            for(int j = 0; j < CollisionShapeType::Count; ++j) {
                m_algorithms[i][j] = getAlgorithm(i, j);
            }
        }
    }

    std::unique_ptr<CollisionAlgorithm> CollisionDispatcher::getAlgorithm(int shapeTypeInt1, int shapeTypeInt2) {
        return std::make_unique<BoxBoxCollisionAlgorithm>();
    }

}