#include "BoundingVolumeTree.h"

namespace redPhysics3d {

    BoundingVolumeTree::BoundingVolumeTree() {
        m_startNode = new BoundingVolumeNode(nullptr);
    }

    BoundingVolumeTree::~BoundingVolumeTree() {
        delete m_startNode;
    }

    void BoundingVolumeTree::getPotentialCollisions(std::vector<PotentialCollision>& potentialCollisions) {
        if(m_startNode) m_startNode->getPotentialCollisions(potentialCollisions);
    }

    void BoundingVolumeTree::insert(CollisionBody* collisionBody) {
        BoundingVolumeNode* newNode = m_startNode->insert(collisionBody, BoundingVolume(collisionBody));
        m_nodeMap.insert(std::pair<CollisionBody*, BoundingVolumeNode*>(collisionBody, newNode));

        // Update the other node that was reallocated
        if(newNode->parent) {
            BoundingVolumeNode* sibling = newNode->parent->children[0];
            auto search = m_nodeMap.find(sibling->collisionBody);
            if(search != m_nodeMap.end()) {
                search->second = sibling;
            }
        }
    }

    void BoundingVolumeTree::remove(CollisionBody* collisionBody) {
        auto search = m_nodeMap.find(collisionBody);
        if(search == m_nodeMap.end()) return;

        BoundingVolumeNode* foundNode = search->second;

        BoundingVolumeNode* parent = foundNode->parent;

        delete search->second;
        m_nodeMap.erase(search);

        // Update the other node that was reallocated
        if(parent && parent->collisionBody) {
            search = m_nodeMap.find(parent->collisionBody);
            if(search != m_nodeMap.end()) {
                search->second = parent;
            }
        }
        else if(search->second == m_startNode) {
            m_startNode = new BoundingVolumeNode(nullptr);
        }
    }

    void BoundingVolumeTree::update(CollisionBody* collisionBody) {
        remove(collisionBody);
        insert(collisionBody);
    }

}