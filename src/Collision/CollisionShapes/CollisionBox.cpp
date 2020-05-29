#include "CollisionBox.h"
#include <cmath>

namespace redPhysics3d {

    CollisionBox::CollisionBox(CollisionBody* collisionBody) : CollisionShape(collisionBody) {
        updateCollisionShape();
    }

    void CollisionBox::updateAABBsize() {
        AABBmin = AABBmax = verticies[0];
        for(int i = 1; i < 8; ++i) {
            if(verticies[i].x < AABBmin.x) AABBmin.x = verticies[i].x;
            if(verticies[i].y < AABBmin.y) AABBmin.y = verticies[i].y;
            if(verticies[i].z < AABBmin.z) AABBmin.z = verticies[i].z;
            if(verticies[i].x > AABBmax.x) AABBmax.x = verticies[i].x;
            if(verticies[i].y > AABBmax.y) AABBmax.y = verticies[i].y;
            if(verticies[i].z > AABBmax.z) AABBmax.z = verticies[i].z;
        }
    }

    std::array<Vector3, 8> CollisionBox::getBoxVerticies() {
        Vector3 hs = size; // * 0.5
        return { getRotationMatrix() * Vector3(hs.x,  hs.y, hs.z), getRotationMatrix() * Vector3(hs.x,  hs.y, -hs.z), getRotationMatrix() * Vector3(-hs.x,  hs.y, -hs.z), getRotationMatrix() * Vector3(-hs.x,  hs.y, hs.z),
                 getRotationMatrix() * Vector3(hs.x, -hs.y, hs.z), getRotationMatrix() * Vector3(hs.x, -hs.y, -hs.z), getRotationMatrix() * Vector3(-hs.x, -hs.y, -hs.z), getRotationMatrix() * Vector3(-hs.x, -hs.y, hs.z)};
    }

    std::array<Vector3, 4> CollisionBox::getFaceVerticies(int index) {
        switch(index) {
        case 0:
            return { verticies[0], verticies[1], verticies[5], verticies[4] }; // Positive x
        case 1:
            return { verticies[0], verticies[1], verticies[2], verticies[3] }; // Positive y
        case 2:
            return { verticies[1], verticies[2], verticies[6], verticies[5] }; // Negative z
        case 3:
            return { verticies[2], verticies[3], verticies[7], verticies[6] }; // Negative x
        case 4:
            return { verticies[4], verticies[5], verticies[6], verticies[7] }; // Negative y
        case 5:
            return { verticies[0], verticies[3], verticies[7], verticies[4] }; // Positive z
        }
        
        return { Vector3(), Vector3(), Vector3(), Vector3() };
    }

    void CollisionBox::updateCollisionShape() {
        verticies = getBoxVerticies();
        updateAABBsize();
    }
    
}