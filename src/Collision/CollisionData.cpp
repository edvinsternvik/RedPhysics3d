#include "CollisionData.h"
#include "../CollisionBody/CollisionBody.h"

namespace redPhysics3d {

    Contact::Contact(CollisionShape* collider1, CollisionShape* collider2, const Vector3& point, const Vector3& collider1Normal, const float& penetration)
        : colliders{collider1, collider2}, contactPoint(point), collider1Normal(collider1Normal), penetration(penetration) {
    }

    void Contact::prepareContact() {
        relativeContactPosition[0] = contactPoint - colliders[0]->getCollisionBody()->position;
        relativeContactPosition[1] = contactPoint - colliders[1]->getCollisionBody()->position;
        worldToContactMatrix = calculateWorldToContactMatrix();
    }

    Matrix3x3 Contact::calculateWorldToContactMatrix() const {
        Vector3 axes[3] = { collider1Normal, Vector3(), Vector3() };
        Vector3 otherAxis = Vector3(1.0, 0.0, 0.0);
        if(abs(axes[0].x > abs(axes[0].y))) otherAxis = Vector3(0.0, 1.0, 0.0);

        axes[1] = axes[0].cross(otherAxis);
        axes[1].normalize();
        axes[2] = axes[0].cross(axes[1]);

        return Matrix3x3(axes[0].x, axes[0].y, axes[0].z, // Transposed to become the inverse of the contact to world matrix
                         axes[1].x, axes[1].y, axes[1].z,
                         axes[2].x, axes[2].y, axes[2].z);
    }

    CollisionData::CollisionData() {
    }

    void CollisionData::addContactPoint(CollisionShape* collider1, CollisionShape* collider2, const Vector3& point, const Vector3& collider1Normal, const float& penetration) {
        contacts.push_back(Contact(collider1, collider2, point, collider1Normal, penetration));
    }

}