#include "BoundingVolumeNode.h"
#include "../../Math/Etc.h"

namespace redPhysics3d {

    BoundingVolumeNode::BoundingVolumeNode(CollisionBody* collisionBody) 
            : boundingVolume(collisionBody), children{nullptr, nullptr}, collisionBody(collisionBody) {
    }

    BoundingVolumeNode::BoundingVolumeNode(CollisionBody* collisionBody, const BoundingVolume&  boundingVolume)
            : boundingVolume(boundingVolume.position, boundingVolume.halfSize), children{nullptr, nullptr}, collisionBody(collisionBody) {
    }

    BoundingVolumeNode::~BoundingVolumeNode() {
        if(parent) {
            BoundingVolumeNode* sibling = parent->children[0] == this ? parent->children[1] : parent->children[0];
            parent->boundingVolume = sibling->boundingVolume;
            parent->collisionBody = sibling->collisionBody;
            parent->children[0] = sibling->children[0];
            parent->children[1] = sibling->children[1];

            sibling->parent = nullptr;
            sibling->collisionBody = nullptr;
            sibling->children[0] = nullptr;
            sibling->children[1] = nullptr;
            delete sibling;

            parent->calculateBoundingVolume();
        }

        if(children[0]) {
            children[0]->parent = nullptr;
            delete children[0];
        }

        if(children[1]) {
            children[1]->parent = nullptr;
            delete children[1];
        }
    }

    void BoundingVolumeNode::getPotentialCollisions(std::vector<PotentialCollision>& potentialCollisions) const {
        if(isLeaf()) return;

        children[0]->getPotentialCollisionsWith(potentialCollisions, children[1]);

        children[0]->getPotentialCollisions(potentialCollisions);
        children[1]->getPotentialCollisions(potentialCollisions);
    }

    void BoundingVolumeNode::getPotentialCollisionsWith(std::vector<PotentialCollision>& potentialCollisions, const BoundingVolumeNode* other) const {
        if(!boundingVolume.intersects(other->boundingVolume)) {
            return;
        }

        if(isLeaf() && other->isLeaf()) {
            potentialCollisions.push_back(PotentialCollision(collisionBody, other->collisionBody));
            return;
        }

        if(other->isLeaf() || (!isLeaf() && boundingVolume.getSize() >= other->boundingVolume.getSize())) {
            children[0]->getPotentialCollisionsWith(potentialCollisions, other);
            children[1]->getPotentialCollisionsWith(potentialCollisions, other);
        }
        else {
            getPotentialCollisionsWith(potentialCollisions, other->children[0]);
            getPotentialCollisionsWith(potentialCollisions, other->children[1]);
        }
    }

    void BoundingVolumeNode::insert(CollisionBody* newBody, const BoundingVolume& newVolume) {
        if(isLeaf() && collisionBody == nullptr) {
            collisionBody = newBody;
            boundingVolume = newVolume;
            return;
        }

        if(isLeaf()) {
            children[0] = new BoundingVolumeNode(collisionBody, boundingVolume);
            children[1] = new BoundingVolumeNode(newBody, newVolume);
            children[0]->parent = this;
            children[1]->parent = this;
            collisionBody = nullptr;
            calculateBoundingVolume();
        }
        else {
            if(children[0]->boundingVolume.getGrowth(newVolume) > children[1]->boundingVolume.getGrowth(newVolume)) {
                children[1]->insert(newBody, newVolume);
            }
            else {
                children[0]->insert(newBody, newVolume);
            }
        }
    }

    void BoundingVolumeNode::calculateBoundingVolume() {
        if(isLeaf()) {
            boundingVolume.calculate(collisionBody);
        }
        else {
            Vector3 l = max(children[0]->boundingVolume.position + children[0]->boundingVolume.halfSize, children[1]->boundingVolume.position + children[1]->boundingVolume.halfSize);
            Vector3 s = min(children[0]->boundingVolume.position - children[0]->boundingVolume.halfSize, children[1]->boundingVolume.position - children[1]->boundingVolume.halfSize);
            boundingVolume.position = (l + s) * 0.5;
            boundingVolume.halfSize = (l - s) * 0.5;
        }

        if(parent) parent->calculateBoundingVolume();
    }


}