#ifndef ARE_INCLUDE_BASIC_VEC3_H
#define ARE_INCLUDE_BASIC_VEC3_H

#include <cmath>
#include <sys/cdefs.h>

namespace are {

class Vec3 {
private:
	double e_[3]; // x, y, z value

public:
	// Constructors
	Vec3();
	Vec3(double e0, double e1, double e2);

	// Copy constructor
	Vec3(const Vec3 &other);

	// Destructor
	~Vec3() = default;

	// Component access
	double *e();
	double x() const;
	double y() const;
	double z() const;

	// Vector operations
	Vec3 operator-() const; // Negation
	double operator[](int i) const;
	double &operator[](int i);

	// Vector assignment
	Vec3 &operator=(const Vec3 &other);

	// Compound assignment operations
	Vec3 &operator+=(const Vec3 &v);
	Vec3 &operator-=(const Vec3 &v);
	Vec3 &operator*=(double t);
	Vec3 &operator/=(double t); // Returns invalid vector (NaN) if division by zero

	// Vector length operations
	double length() const;
	double length_squared() const;

	// Normalization - returns invalid vector (NaN) if length is zero
	Vec3 &normalize();
	Vec3 normalized() const;

	// Dot product
	double dot(const Vec3 &v) const;

	// Cross product
	Vec3 cross(const Vec3 &v) const;

	// Check if vector is near zero
	bool near_zero() const;

	// Friend function declarations
	friend Vec3 operator+(const Vec3 &u, const Vec3 &v);
	friend Vec3 operator-(const Vec3 &u, const Vec3 &v);
	friend Vec3 operator*(const Vec3 &u, const Vec3 &v);
	friend Vec3 operator*(double t, const Vec3 &v);
	friend Vec3 operator*(const Vec3 &v, double t);
	friend Vec3 operator/(const Vec3 &v, double t); // Returns invalid vector (NaN) if division by zero
};

// Vector operation friend function declarations
Vec3 operator+(const Vec3 &u, const Vec3 &v);
Vec3 operator-(const Vec3 &u, const Vec3 &v);
Vec3 operator*(const Vec3 &u, const Vec3 &v);
Vec3 operator*(double t, const Vec3 &v);
Vec3 operator*(const Vec3 &v, double t);
Vec3 operator/(const Vec3 &v, double t); // Returns invalid vector (NaN) if division by zero

// Utility functions
Vec3 reflect(const Vec3 &v, const Vec3 &n);
Vec3 refract(const Vec3 &uv, const Vec3 &n, double etai_over_etat); // Returns invalid vector (NaN) if total internal reflection occurs

// Some other type based on Vec3 class
using Point3 = Vec3;
using Color3 = Vec3;

}

#endif
