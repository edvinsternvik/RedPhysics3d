#pragma once
#include <ostream>

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

		Vector3 operator*(const Vector3& vec3) const {
			return Vector3(this->x * vec3.x, this->y * vec3.y, this->z * vec3.z);
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