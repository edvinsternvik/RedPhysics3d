#include "CollisionAlgorithm.h"
#include "../CollisionShapes/CollisionBox.h"
#include "../../Math/Etc.h"
#include "../../Math/Matrix3x3.h"
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
        std::vector<Vector3> axes = {box1->getRotationMatrix() * Vector3(1.0,0,0), box1->getRotationMatrix() * Vector3(0,1.0,0), box1->getRotationMatrix() * Vector3(0,0,-1.0),
                                     box2->getRotationMatrix() * Vector3(1.0,0,0), box2->getRotationMatrix() * Vector3(0,1.0,0), box2->getRotationMatrix() * Vector3(0,0,-1.0)};


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

        float referenceFaceSignMultiplier = axesSeparationDepths[depthIndex] < 0.0 ? -1.0 : 1.0;
        float penetrationDepth = std::abs(axesSeparationDepths[depthIndex]);
        collisionData.setCollisionData(axes[depthIndex] * referenceFaceSignMultiplier, penetrationDepth);

        bool isFaceAxis = depthIndex < 6;
        if(isFaceAxis) {
            // Calculate face vs face, face vs edge and face vs vertex contact points

            bool isBox1Reference = depthIndex < 3;

            Vector3 referenceFaceNormal = axes[depthIndex] * referenceFaceSignMultiplier * (isBox1Reference ? -1 : 1);
            CollisionBox* referenceShape = isBox1Reference ? box1 : box2; // Reference shape is the shape that has a colliding face
            CollisionBox* incidentShape =  isBox1Reference ? box2 : box1; // Incident shape is the other shape

            // Get all face normals for the incident shape
            int a = isBox1Reference ? 3 : 0;
            Vector3 incidentFaceNormals[6] = { axes[a], axes[a + 1], axes[a + 2], axes[a] * -1, axes[a + 1] * -1, axes[a + 2] * -1 };

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
            Vector3 localReferenceFaceNormal = referenceShape->getInvertedRotationMatrix() * referenceFaceNormal;
            if(localReferenceFaceNormal.x > 0.9) referenceShapeRotateUpMatrix = Matrix3x3::getRotationMatrixZ(hpi);
            else if(localReferenceFaceNormal.x < -0.9) referenceShapeRotateUpMatrix = Matrix3x3::getRotationMatrixZ(-hpi);
            else if(localReferenceFaceNormal.y < -0.9) referenceShapeRotateUpMatrix = Matrix3x3::getRotationMatrixZ(2 * hpi);
            else if(localReferenceFaceNormal.z >  0.9) referenceShapeRotateUpMatrix = Matrix3x3::getRotationMatrixX(-hpi);
            else if(localReferenceFaceNormal.z < -0.9) referenceShapeRotateUpMatrix = Matrix3x3::getRotationMatrixX(hpi);
            Matrix3x3 referenceFaceLocalSpaceMatrix = referenceShapeRotateUpMatrix * referenceShape->getInvertedRotationMatrix();
            localReferenceFaceNormal = referenceFaceLocalSpaceMatrix * referenceFaceNormal;

            // Transform the verticies in the incident face to the reference face local space
            Vector3 deltaPos = (incidentShape->getPosition() - referenceShape->getPosition());
            deltaPos.x -= referenceFaceNormal.x * referenceShape->size.x;
            deltaPos.y -= referenceFaceNormal.y * referenceShape->size.y;
            deltaPos.z -= referenceFaceNormal.z * referenceShape->size.z;
            auto incidentVerticies = incidentShape->getFaceVerticies(incidentFaceIndex);
            for(Vector3& incidentVertex : incidentVerticies) {
                incidentVertex = incidentVertex + deltaPos;
                incidentVertex = referenceFaceLocalSpaceMatrix * incidentVertex;
            }

            // Iterate over the edges in the incident face and store points that intersect the reference face side planes
            Vector3 bounds = referenceShapeRotateUpMatrix * referenceShape->size;
            for(int i = 0; i < 4; ++i) {
                Vector3& vertex1 = incidentVerticies[i];
                Vector3& vertex2 = incidentVerticies[(i + 1) % 4];
                
                getClippedVerticies(vertex1, vertex2, bounds, collisionData);
                getClippedVerticies(vertex1, vertex2, bounds * -1, collisionData);
            }

            // Transform the verticies back into world space
            Matrix3x3 referenceFaceLocalSpaceMatrixInv = referenceFaceLocalSpaceMatrix.inverse();
            for(auto& v : collisionData.contacts) v.contactPoint = (referenceFaceLocalSpaceMatrixInv * v.contactPoint) - deltaPos + incidentShape->getPosition();

            // Store incident face vertecies colliding with the reference shape
            bool incidentVerticiesColliding = false;
            bounds = referenceShape->size;
            for(Vector3& vertex : incidentVerticies) {
                if(vertex.y <= 0.0 && vertex.y >= -bounds.y * 2 && vertex.x <= bounds.x && vertex.x >= -bounds.x && vertex.z <= bounds.z && vertex.z >= -bounds.z) {
                    // Transform vertex into world space
                    Vector3 vertexWorld = (referenceFaceLocalSpaceMatrixInv * vertex) - deltaPos + incidentShape->getPosition();

                    // Check if contact already exists
                    bool exists = false;
                    for(int i = 0; i < collisionData.contacts.size(); ++i) {
                        if((vertexWorld - collisionData.contacts[i].contactPoint).magnitudeSquare() < 0.001) {
                            exists = true;
                            break;
                        }
                    }

                    // Add contact if it doesn't already exist
                    if(!exists) {
                        collisionData.addContactPoint(vertexWorld, -vertex.y);
                        incidentVerticiesColliding = true;
                    }
                }
            }          

            if(incidentVerticiesColliding) {
                // Store reference face vertecies colliding with the incident shape
                std::array<Vector3, 8> referenceVerticies = referenceShape->verticies;
                bounds = incidentShape->size;
                for(Vector3& vertex : referenceVerticies) {
                    Vector3 rVertex = vertex + referenceShape->getPosition() - incidentShape->getPosition();
                    rVertex = incidentShape->getInvertedRotationMatrix() * rVertex;
                    
                    if(rVertex.x <= bounds.x && rVertex.x >= -bounds.x && rVertex.y <= bounds.y && rVertex.y >= -bounds.y && rVertex.z <= bounds.z && rVertex.z >= -bounds.z) {
                        // Check if contact already exists
                        bool exist = false;
                        Vector3 contactPointWorld = vertex + referenceShape->getPosition();
                        for(int i = 0; i < collisionData.contacts.size(); ++i) {
                            if((contactPointWorld - collisionData.contacts[i].contactPoint).magnitudeSquare() < 0.001) {
                                exist = true;
                                break;
                            }
                        }
                        if(!exist) {
                            // Calculate penetration
                            Vector3 normal = referenceShape->getInvertedRotationMatrix() * collisionData.collider1Normal;
                            if(normal.x < 0.0) bounds.x = -bounds.x;
                            if(normal.y < 0.0) bounds.y = -bounds.y;
                            if(normal.z < 0.0) bounds.z = -bounds.z;

                            float dx = (bounds.x - rVertex.x) / normal.x;
                            float dy = (bounds.y - rVertex.y) / normal.y;
                            float dz = (bounds.z - rVertex.z) / normal.z;
                            float penetration = std::min(dx, std::min(dy, dz));

                            collisionData.addContactPoint(contactPointWorld, collisionData.depth);
                        }
                    }
                }
            }

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
            collisionData.addContactPoint(smallestV, penetrationDepth);
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
                collisionData.addContactPoint(clippedVertex, -clippedVertex.y);
            }
        }

        if((vertex1.z < bounds.z && vertex2.z > bounds.z) || (vertex1.z > bounds.z && vertex2.z < bounds.z)) {
            float dxDz = deltaEdge.z == 0 ? 0.0 : deltaEdge.x / deltaEdge.z;
            float dyDz = deltaEdge.z == 0 ? 0.0 : deltaEdge.y / deltaEdge.z;
            
            float newZ = bounds.z;
            Vector3 clippedVertex(newZ * dxDz + vertex2.x - vertex2.z * dxDz, newZ * dyDz + vertex2.y - vertex2.z * dyDz, newZ);
            if(clippedVertex.y <= 0.0 && std::abs(clippedVertex.x) <= std::abs(bounds.x)) {
                collisionData.addContactPoint(clippedVertex, -clippedVertex.y);
            }
        }
    }

}