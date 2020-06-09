#pragma once
#include "Vector.h"

namespace redPhysics3d {
    
    template <typename T> int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }

    Vector3 max(const Vector3& vec1, const Vector3& vec2);
    Vector3 min(const Vector3& vec1, const Vector3& vec2);

    bool computeClosestPointBetweenEdges(const Vector3& edge1PointA, const Vector3& edge1PointB, const Vector3& edge2PointA, const Vector3& edge2PointB, Vector3& closestPointEdge1, Vector3& closestPointEdge2);
    
}