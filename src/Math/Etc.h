#pragma once
#include "Vector.h"

namespace redPhysics3d {
    
    template <typename T> int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }

    bool computeClosestPointBetweenEdges(const Vector3& edge1PointA, const Vector3& edge1PointB, const Vector3& edge2PointA, const Vector3& edge2PointB, Vector3& closestPointEdge1, Vector3& closestPointEdge2);
    
}