#include "CollisionAlgorithm.h"
#include "../CollisionShapes/CollisionBox.h"
#include <vector>

namespace redPhysics3d {

    void getMinMax(CollisionBox* cb, Vector3& axis, float& min, float& max);

    bool BoxBoxCollisionAlgorithm::testCollision(CollisionShape* shape1, CollisionShape* shape2) {
        CollisionBox* box1 = (CollisionBox*)shape1, *box2 = (CollisionBox*)shape2;
        
        std::vector<std::pair<float, Vector3>> smallestDepths;
        std::vector<Vector3> axes = {Vector3(1.0,0,0) * box1->getRotationMatrix(), Vector3(0,1.0,0) * box1->getRotationMatrix(), Vector3(0,0,-1.0) * box1->getRotationMatrix(),
                                     Vector3(1.0,0,0) * box2->getRotationMatrix(), Vector3(0,1.0,0) * box2->getRotationMatrix(), Vector3(0,0,-1.0) * box2->getRotationMatrix()};

        for(int i = 0; i < 3; ++i) {
            for(int j = 3; j < 6; ++j) {
                Vector3 cross = axes[i].cross(axes[j]);
                if(cross.x != 0 || cross.y != 0 || cross.z != 0) {
                    cross.normalize();
                    axes.push_back(cross);
                }
            }
        }

        for(Vector3& axis : axes) {
            float min1, max1, min2, max2;
            getMinMax(box1, axis, min1, max1);
            getMinMax(box2, axis, min2, max2);

            if(min1 > max2 || min2 > max1) return false;

            float smallest = std::abs(min2 - max1) < std::abs(max2 - min1) ? min2 - max1 : max2 - min1;
            smallestDepths.push_back(std::pair<float, Vector3>(smallest, axis * smallest));
        }

        int depthId = 0;
        for(int i = 0; i < smallestDepths.size(); ++i) {
            if(smallestDepths[i].first != 0.0 && std::abs(smallestDepths[i].first) < std::abs(smallestDepths[depthId].first)) depthId = i;
        }

        Vector3 depth = smallestDepths[depthId].second * 1.001;

        box1->setPosition(box1->getPosition() + depth);

        return true;
    }

    void getMinMax(CollisionBox* cb, Vector3& axis, float& min, float& max) {
        for(int i = 0; i < 8; ++i) {
            Vector3 vertex = cb->verticies[i] + cb->getPosition();
            float projectionOnAxis = vertex.dot(axis); // Axis must be normalized

            if(i == 0) min = max = projectionOnAxis;
            else {
                min = std::min(min, projectionOnAxis);
                max = std::max(max, projectionOnAxis);
            }
        }
    }

}