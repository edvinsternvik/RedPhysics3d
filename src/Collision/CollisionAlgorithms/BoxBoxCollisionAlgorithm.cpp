#include "CollisionAlgorithm.h"
#include "../CollisionShapes/CollisionBox.h"
#include "../../Math/Etc.h"
#include <vector>
#include <limits>

#define hpi 1.570796

namespace redPhysics3d {

    void getMinMax(CollisionBox* cb, Vector3& axis, float& min, float& max);
    void getClippedVerticies(const Vector3& vertex1, const Vector3& vertex2, const Vector3& bounds, CollisionData& collisionData);

    CollisionData BoxBoxCollisionAlgorithm::testCollision(CollisionShape* shape1, CollisionShape* shape2) {
        CollisionBox* box1 = (CollisionBox*)shape1, *box2 = (CollisionBox*)shape2;
        
        std::vector<float> axesSeparationDepths;

        // Calculate the face axes
        std::vector<Vector3> axes = {Vector3(1.0,0,0) * box1->getRotationMatrix(), Vector3(0,1.0,0) * box1->getRotationMatrix(), Vector3(0,0,1.0) * box1->getRotationMatrix(),
                                     Vector3(1.0,0,0) * box2->getRotationMatrix(), Vector3(0,1.0,0) * box2->getRotationMatrix(), Vector3(0,0,1.0) * box2->getRotationMatrix()};


        // Calculate the edge axes
        for(int i = 0; i < 3; ++i) {
            for(int j = 3; j < 6; ++j) {
                Vector3 cross = axes[i].cross(axes[j]);
                if(cross.x != 0 || cross.y != 0 || cross.z != 0) {
                    cross.normalize();
                    axes.push_back(cross);
                }
            }
        }

        CollisionData collisionData(false, shape1, shape2);

        for(int i = 0; i < axes.size(); ++i) {
            // Get the bounds of the shapes when projected onto the axis
            float min1, max1, min2, max2;
            getMinMax(box1, axes[i], min1, max1);
            getMinMax(box2, axes[i], min2, max2);

            // Found a separating axis -> no collision
            if(min1 > max2 || min2 > max1) return collisionData;  

            float smallest = std::abs(min2 - max1) < std::abs(max2 - min1) ? min2 - max1 : max2 - min1;
            axesSeparationDepths.push_back(smallest);
        }

        // No separating axis was found -> collision

        // Get the axis with the smallest penetration depth
        int depthIndex = 0;
        for(int i = 0; i < axesSeparationDepths.size(); ++i) {
            if(std::abs(axesSeparationDepths[i]) < std::abs(axesSeparationDepths[depthIndex])) depthIndex = i;
        }

        collisionData.setCollisionData(axes[depthIndex], axesSeparationDepths[depthIndex]);

        bool isFaceAxis = depthIndex < 6;
        if(isFaceAxis) {
            // Calculate face vs face, face vs edge and face vs vertex contact points

            bool isBox1Reference = depthIndex < 3;

            Vector3 referenceFaceNormal = axes[depthIndex] * sgn(axesSeparationDepths[depthIndex]) * (isBox1Reference ? -1 : 1);
            CollisionBox* referenceShape = isBox1Reference ? box1 : box2; // Reference shape is the shape that has a colliding face
            CollisionBox* incidentShape =  isBox1Reference ? box2 : box1; // Incident shape is the other shape

            // Get all face normals for the incident shape
            Vector3 incidentFaceNormals[6] = { axes[0], axes[1], axes[2], axes[0] * -1, axes[1] * -1, axes[2] * -1 };
            if(isBox1Reference) { 
                incidentFaceNormals[0] = axes[3]; incidentFaceNormals[1] = axes[4]; incidentFaceNormals[2] = axes[5]; incidentFaceNormals[3] = axes[3] * -1; incidentFaceNormals[4] = axes[4] * -1; incidentFaceNormals[5] = axes[5] * -1; 
            }

            // Get the incident face, which is the face most antiparallel to the reference face
            int incidentFaceIndex = 0;
            float smallestDot = std::numeric_limits<float>::max();
            for(int i = 0; i < 6; ++i) {
                float dot = incidentFaceNormals[i].dot(referenceFaceNormal);
                if(dot <= smallestDot) {
                    smallestDot = dot;
                    incidentFaceIndex = i;
                }
            }
            
            // Calculate a rotation matrix that rotates all points to local reference face space, where the reference face normal points up
            Matrix3x3 referenceShapeRotateUpMatrix(1,0,0, 0,1,0, 0,0,1);
            Vector3 localReferenceFaceNormal = referenceFaceNormal * referenceShape->getInvertedRotationMatrix();
            if(localReferenceFaceNormal.x > 0.9) referenceShapeRotateUpMatrix *= Matrix3x3::getRotationMatrixZ(hpi);
            else if(localReferenceFaceNormal.x < -0.9) referenceShapeRotateUpMatrix *= Matrix3x3::getRotationMatrixZ(-hpi);
            else if(localReferenceFaceNormal.y < -0.9) referenceShapeRotateUpMatrix *= Matrix3x3::getRotationMatrixZ(2 * hpi);
            else if(localReferenceFaceNormal.z >  0.9) referenceShapeRotateUpMatrix *= Matrix3x3::getRotationMatrixX(-hpi);
            else if(localReferenceFaceNormal.z < -0.9) referenceShapeRotateUpMatrix *= Matrix3x3::getRotationMatrixX(hpi);
            Matrix3x3 referenceFaceLocalSpaceMatrix = referenceShape->getInvertedRotationMatrix() * referenceShapeRotateUpMatrix;
            localReferenceFaceNormal = referenceFaceNormal * referenceFaceLocalSpaceMatrix;

            // Transform the verticies in the incident face to the reference face local space
            Vector3 deltaPos = (incidentShape->getPosition() - referenceShape->getPosition());
            deltaPos.x -= referenceFaceNormal.x * referenceShape->getSize().x;
            deltaPos.y -= referenceFaceNormal.y * referenceShape->getSize().y;
            deltaPos.z -= referenceFaceNormal.z * referenceShape->getSize().z;
            auto incidentVerticies = incidentShape->getFaceVerticies(incidentFaceIndex);
            for(Vector3& incidentVertex : incidentVerticies) {
                incidentVertex = incidentVertex + deltaPos;
                incidentVertex *= referenceFaceLocalSpaceMatrix;
            }

            // Iterate over the edges in the incident face and store points that intersect the reference face side planes
            Vector3 bounds = referenceShape->getSize() * referenceShapeRotateUpMatrix;
            for(int i = 0; i < 4; ++i) {
                Vector3& vertex1 = incidentVerticies[i];
                Vector3& vertex2 = incidentVerticies[(i + 1) % 4];
                
                getClippedVerticies(vertex1, vertex2, bounds, collisionData);
                getClippedVerticies(vertex1, vertex2, bounds * -1, collisionData);
            }

            // Also store incident face vertecies colliding with the reference shape
            bounds = referenceShape->getSize();
            for(Vector3& vertex : incidentVerticies) {
                if(vertex.y <= 0.0 && vertex.x <= bounds.x && vertex.x >= -bounds.x && vertex.z <= bounds.z && vertex.z >= -bounds.z) {
                    collisionData.addContactPoint(vertex);
                }
            }

            // Transform the verticies back into world space
            Matrix3x3 referenceFaceLocalSpaceMatrixInv = referenceFaceLocalSpaceMatrix.inverse();
            for(Vector3& v : collisionData.contactPoints) v = (v * referenceFaceLocalSpaceMatrixInv) - deltaPos + incidentShape->getPosition();
        }
        else {
            // Calculate contact points for edge vs edge collision

            // Todo: get colliding edges directly instead of testing each edge pair

            // Get edge pairs
            std::vector<std::pair<Vector3,Vector3>> b1Edges, b2Edges;

            for(int i = 0; i < 4; ++i) {
                int n = (i+1)%4;
                b1Edges.push_back(std::pair<Vector3,Vector3>(box1->verticies[i], box1->verticies[n]));
                b2Edges.push_back(std::pair<Vector3,Vector3>(box2->verticies[i], box2->verticies[n]));

                int j = i + 4;
                int m = n + 4;
                b1Edges.push_back(std::pair<Vector3,Vector3>(box1->verticies[j], box1->verticies[m]));
                b2Edges.push_back(std::pair<Vector3,Vector3>(box2->verticies[j], box2->verticies[m]));

                b1Edges.push_back(std::pair<Vector3,Vector3>(box1->verticies[i], box1->verticies[j]));
                b2Edges.push_back(std::pair<Vector3,Vector3>(box2->verticies[i], box2->verticies[j]));
            }

            // Calculate edge pair with smallest distance between them
            Vector3 smallestV;
            float smallest = std::numeric_limits<float>::max();
            for(auto& b1E : b1Edges) {
                for(auto& b2E : b2Edges) {
                    Vector3 res1, res2;
                    if(computeClosestPointBetweenEdges(b1E.first + box1->getPosition(), b1E.second + box1->getPosition(), b2E.first + box2->getPosition(), b2E.second + box2->getPosition(), res1, res2)) {
                        Vector3 delta = res2 - res1;
                        if(delta.magnitudeSquare() < smallest) {
                            smallest = delta.magnitudeSquare();   
                            smallestV = res2;
                        }
                    }
                }
            }

            collisionData.addContactPoint(smallestV);
        }

        return collisionData;
    }
    
    // Projects the collisionbox's verticies onto the axis and returns the minimum and maximum lengths
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

    // 
    void getClippedVerticies(const Vector3& vertex1, const Vector3& vertex2, const Vector3& bounds, CollisionData& collisionData) {
        Vector3 deltaEdge = vertex1 - vertex2;

        if((vertex1.x < bounds.x && vertex2.x > bounds.x) || (vertex1.x > bounds.x && vertex2.x < bounds.x)) {
            float dyDx = deltaEdge.x == 0 ? 0.0 : deltaEdge.y / deltaEdge.x;
            float dzDx = deltaEdge.x == 0 ? 0.0 : deltaEdge.z / deltaEdge.x;
            
            float newX = bounds.x;
            Vector3 clippedVertex(newX, newX * dyDx + vertex2.y - vertex2.x * dyDx, newX * dzDx + vertex2.z - vertex2.x * dzDx);
            if(clippedVertex.y <= 0.0 && std::abs(clippedVertex.z) <= std::abs(bounds.z)) {
                collisionData.addContactPoint(clippedVertex);
            }
        }

        if((vertex1.z < bounds.z && vertex2.z > bounds.z) || (vertex1.z > bounds.z && vertex2.z < bounds.z)) {
            float dxDz = deltaEdge.z == 0 ? 0.0 : deltaEdge.x / deltaEdge.z;
            float dyDz = deltaEdge.z == 0 ? 0.0 : deltaEdge.y / deltaEdge.z;
            
            float newZ = bounds.z;
            Vector3 clippedVertex(newZ * dxDz + vertex2.x - vertex2.z * dxDz, newZ * dyDz + vertex2.y - vertex2.z * dyDz, newZ);
            if(clippedVertex.y <= 0.0 && std::abs(clippedVertex.x) <= std::abs(bounds.x)) {
                collisionData.addContactPoint(clippedVertex);
            }
        }
    }

}