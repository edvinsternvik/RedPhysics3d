#include "CollisionAlgorithm.h"
#include "../CollisionShapes/CollisionBox.h"

namespace redPhysics3d {

    bool BoxBoxCollisionAlgorithm::testCollision(CollisionShape* shape1, CollisionShape* shape2) {
        CollisionBox* box1 = (CollisionBox*)shape1, *box2 = (CollisionBox*)shape2;

        for(int iterations = 0; iterations < 2; ++iterations) {
            Vector3 deltaPos = box2->getPosition() - box1->getPosition();
            Vector3 min, max;
            for(int i = 0; i < 8; ++i) {
                Vector3 rotatedVertex = (box2->verticies[i]) + deltaPos;

                rotatedVertex = rotatedVertex * box1->getInvertedRotationMatrix();

                if(i == 0) min = max = rotatedVertex;
                else {
                    min.x = std::min(min.x, rotatedVertex.x);
                    min.y = std::min(min.y, rotatedVertex.y);
                    min.z = std::min(min.z, rotatedVertex.z);

                    max.x = std::max(max.x, rotatedVertex.x);
                    max.y = std::max(max.y, rotatedVertex.y);
                    max.z = std::max(max.z, rotatedVertex.z);
                }
            }

            Vector3 hs = box1->getSize();
            if(max.x < -hs.x || min.x > hs.x || max.y < -hs.y || min.y > hs.y || max.z < -hs.z || min.z > hs.z) return false;

            CollisionBox* temp = box1;
            box1 = box2;
            box2 = temp;
        }

        return true;
    }

}