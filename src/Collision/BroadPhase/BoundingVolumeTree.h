#pragma once
#include <unordered_map>
#include <vector>
#include "BoundingVolumeNode.h"

namespace redPhysics3d {

    class CollisionBody;

    class BoundingVolumeTree {
    public:
        BoundingVolumeTree();

        void getPotentialCollisions(std::vector<PotentialCollision>& potentialCollisions);

        void insert(CollisionBody* collisionBody);
        void remove(CollisionBody* collisionBody);
        void update(CollisionBody* collisionBody);

    private:
        BoundingVolumeNode* m_startNode;
        std::unordered_map<CollisionBody*, BoundingVolumeNode*> m_nodeMap;
    };

}