#pragma once
#include "BoundingVolume.h"

#include <vector>

namespace redPhysics3d {
    class CollisionBody;

    class PotentialCollision {
    public:
        PotentialCollision(CollisionBody* body1, CollisionBody* body2) : bodies{body1, body2} {}

        CollisionBody* bodies[2];
    };

    class BoundingVolumeNode {
    public:
        BoundingVolumeNode(CollisionBody* collisionBody);
        BoundingVolumeNode(CollisionBody* collisionBody, const BoundingVolume&  boundingVolume);
        ~BoundingVolumeNode();

        void getPotentialCollisions(std::vector<PotentialCollision>& potentialCollisions) const;

        inline bool isLeaf() const { return children[0] == nullptr && children[1] == nullptr; }
        BoundingVolumeNode* insert(CollisionBody* newBody, const BoundingVolume& newVolume);

    private:
        void calculateBoundingVolume();
        void getPotentialCollisionsWith(std::vector<PotentialCollision>& potentialCollisions, const BoundingVolumeNode* other) const;

    public:
        BoundingVolume boundingVolume;
        BoundingVolumeNode* children[2];
        CollisionBody* collisionBody; // nullptr unless leaf node
        BoundingVolumeNode* parent = nullptr;
    };

}