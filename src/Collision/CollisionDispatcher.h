#pragma once
#include "CollisionAlgorithms/CollisionAlgorithm.h"
#include <memory>

namespace redPhysics3d {

    class CollisionDispatcher {
    public:
        CollisionDispatcher();

        CollisionAlgorithm* getCollisionAlgorithm(CollisionShapeType shapeType1, CollisionShapeType shapeType2);

    private:
        void initAlgorithmMatrix();
        std::unique_ptr<CollisionAlgorithm> getAlgorithm(int shapeTypeInt1, int shapeTypeInt2);

    private:
        std::unique_ptr<CollisionAlgorithm> m_algorithms[CollisionShapeType::Count][CollisionShapeType::Count];
    }; 

}