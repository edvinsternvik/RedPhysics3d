#include "Etc.h"
#include <cmath>
#include <algorithm>

namespace redPhysics3d {

    Vector3 max(const Vector3& vec1, const Vector3& vec2) {
        return Vector3(std::max(vec1.x, vec2.x), std::max(vec1.y, vec2.y), std::max(vec1.z, vec2.z));
    }

    Vector3 min(const Vector3& vec1, const Vector3& vec2) {
        return Vector3(std::min(vec1.x, vec2.x), std::min(vec1.y, vec2.y), std::min(vec1.z, vec2.z));
    }

    // Calculates the closest points on two lines by finding the points p and q, where pq is perpendicular to edge1 and edge2
    bool computeClosestPointBetweenEdges(const Vector3& edge1PointA, const Vector3& edge1PointB, const Vector3& edge2PointA, const Vector3& edge2PointB, Vector3& closestPointEdge1, Vector3& closestPointEdge2) {
        Vector3 edge1 = edge1PointB - edge1PointA;
        Vector3 edge2 = edge2PointB - edge2PointA;

        // p1 = edge1PointA + edge1 * t
        // p2 = edge2PointA + edge2 * s
        // dp = p2 - p1
        // edge1.dot(dp) == 0 --> dp is perpendicular to edge1
        // edge2.dot(dp) == 0 --> dp is perpendicular to edge2

        float e1lenSquare = edge1.magnitudeSquare();
        float e2lenSquare = edge2.magnitudeSquare();
        float e1DotE2 = edge1.dot(edge2);
        if(e1DotE2 == 0.0) e1DotE2 = 0.00001;

        float n1 = edge2.dot(edge2PointA) - edge2.dot(edge1PointA);
        float n2 = edge1.dot(edge2PointA) - edge1.dot(edge1PointA);

        float s = (n2 * e1DotE2 - n1 * e1lenSquare) / (e2lenSquare * e1lenSquare - e1DotE2 * e1DotE2);
        float t = (s * e2lenSquare + n1) / e1DotE2;

        // Clamp values
        float tLim = (edge1PointB.x - edge1PointA.x) / edge1.x;
        float sLim = (edge2PointB.x - edge2PointA.x) / edge2.x;

        if(tLim < 0) {
            if(t > 0) t = 0;
            if(t < tLim) t = tLim;
        }
        else {
            if(t < 0) t = 0;
            if(t > tLim) t = tLim;
        }
        if(sLim < 0) {
            if(s > 0) t = 0;
            if(s < sLim) s = sLim;
        }
        else {
            if(s < 0) s = 0;
            if(s > sLim) s = sLim;
        }

        closestPointEdge1 = edge1PointA + edge1 * t;
        closestPointEdge2 = edge2PointA + edge2 * s;
        return true;
    }

}