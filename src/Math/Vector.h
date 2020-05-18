#pragma once
#include <ostream>
#include "Matrix3x3.h"

namespace redPhysics3d {

	class Vector2 {
	public:
		Vector2(float x, float y);
		Vector2();

		Vector2 operator+(const Vector2& other) {
			return Vector2(this->x + other.x, this->y + other.y);
		}

		Vector2 operator-(const Vector2& other) {
			return Vector2(this->x - other.x, this->y - other.y);
		}

		Vector2 operator*(const float& f) {
			return Vector2(this->x * f, this->y * f);
		}

		Vector2 operator/(const float& f) {
			return Vector2(this->x / f, this->y / f);
		}

		friend std::ostream& operator<<(std::ostream& os, const Vector2& v) {
			return os << "{ " << v.x << ", " << v.y << " }";
		}

	public:
		float x, y;
	};

	class Vector3 {
	public:
		Vector3(const float& x, const float& y, const float& z);
		Vector3();

		float dot(const Vector3& other) const;
		Vector3 cross(const Vector3& other) const;
		float magnitude() const;
		float magnitudeSquare() const;
		void normalize();

		Vector3 operator+(const Vector3& other) const {
			return Vector3(this->x + other.x, this->y + other.y, this->z + other.z);
		}

		Vector3 operator-(const Vector3& other) const {
			return Vector3(this->x - other.x, this->y - other.y, this->z - other.z);
		}

		Vector3 operator*(const float& f) const {
			return Vector3(this->x * f, this->y * f, this->z * f);
		}

		Vector3 operator/(const float& f) const {
			return Vector3(this->x / f, this->y / f, this->z / f);
		}

		Vector3 operator*(const Matrix3x3& mat3) const {
			return Vector3(mat3.at(0, 0) * this->x + mat3.at(0, 1) * this->y + mat3.at(0, 2) * this->z,
						   mat3.at(1, 0) * this->x + mat3.at(1, 1) * this->y + mat3.at(1, 2) * this->z,
						   mat3.at(2, 0) * this->x + mat3.at(2, 1) * this->y + mat3.at(2, 2) * this->z);
		}

		Vector3& operator*=(const Matrix3x3& mat3) {
			float tx = this->x, ty = this->y, tz = this->z;
			this->x = mat3.at(0, 0) * tx + mat3.at(0, 1) * ty + mat3.at(0, 2) * tz;
			this->y = mat3.at(1, 0) * tx + mat3.at(1, 1) * ty + mat3.at(1, 2) * tz;
			this->z = mat3.at(2, 0) * tx + mat3.at(2, 1) * ty + mat3.at(2, 2) * tz;

			return *this;
		}

		Vector3& operator+=(const Vector3& vec3) {
			this->x += vec3.x;
			this->y += vec3.y;
			this->z += vec3.z;

			return *this;
		}

		Vector3& operator-=(const Vector3& vec3) {
			this->x -= vec3.x;
			this->y -= vec3.y;
			this->z -= vec3.z;

			return *this;
		}

		friend std::ostream& operator<<(std::ostream& os, const Vector3& v) {
			return os << "{ " << v.x << ", " << v.y << ", " << v.z << " }";
		}

	public:
		float x, y, z;
	};


}