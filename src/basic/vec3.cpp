#include <basic/vec3.h>

namespace are {

// Default constructor - creates zero vector
Vec3::Vec3()
	: e_ { 0, 0, 0 } {
}

// Parameterized constructor
Vec3::Vec3(double e0, double e1, double e2)
	: e_ { e0, e1, e2 } {
}

// Copy constructor
Vec3::Vec3(const Vec3 &other)
	: e_ { other.e_[0], other.e_[1], other.e_[2] } {
}

// Get e_ array
double *Vec3::e() {
	return e_;
}
// Get x component
double Vec3::x() const {
	return e_[0];
}

// Get y component
double Vec3::y() const {
	return e_[1];
}

// Get z component
double Vec3::z() const {
	return e_[2];
}

// Negation operator
Vec3 Vec3::operator-() const {
	return Vec3(-e_[0], -e_[1], -e_[2]);
}

// Const index access
double Vec3::operator[](int i) const {
	return e_[i];
}

// Non-const index access
double &Vec3::operator[](int i) {
	return e_[i];
}

// Assignment operator
Vec3 &Vec3::operator=(const Vec3 &other) {
	if (this != &other) {
		e_[0] = other.e_[0];
		e_[1] = other.e_[1];
		e_[2] = other.e_[2];
	}
	return *this;
}

// Compound addition assignment
Vec3 &Vec3::operator+=(const Vec3 &v) {
	e_[0] += v.e_[0];
	e_[1] += v.e_[1];
	e_[2] += v.e_[2];
	return *this;
}

// Compound subtraction assignment
Vec3 &Vec3::operator-=(const Vec3 &v) {
	e_[0] -= v.e_[0];
	e_[1] -= v.e_[1];
	e_[2] -= v.e_[2];
	return *this;
}

// Compound multiplication assignment
Vec3 &Vec3::operator*=(double t) {
	e_[0] *= t;
	e_[1] *= t;
	e_[2] *= t;
	return *this;
}

// Compound division assignment - returns invalid vector (NaN) if division by
// zero
Vec3 &Vec3::operator/=(double t) {
	if (t == 0.0) {
		e_[0] = e_[1] = e_[2] = NAN;
		return *this;
	}
	e_[0] /= t;
	e_[1] /= t;
	e_[2] /= t;
	return *this;
}

// Calculate vector length
double Vec3::length() const {
	return std::sqrt(length_squared());
}

// Calculate squared vector length
double Vec3::length_squared() const {
	return e_[0] * e_[0] + e_[1] * e_[1] + e_[2] * e_[2];
}

// Normalize this vector - returns invalid vector (NaN) if length is zero
Vec3 &Vec3::normalize() {
	double len = length();
	if (len == 0.0) {
		e_[0] = e_[1] = e_[2] = NAN;
		return *this;
	}
	*this /= len;
	return *this;
}

// Return normalized copy - returns invalid vector (NaN) if length is zero
Vec3 Vec3::normalized() const {
	double len = length();
	if (len == 0.0) {
		return Vec3(NAN, NAN, NAN);
	}
	return *this / len;
}

// Dot product with another vector
double Vec3::dot(const Vec3 &v) const {
	return e_[0] * v.e_[0] + e_[1] * v.e_[1] + e_[2] * v.e_[2];
}

// Cross product with another vector
Vec3 Vec3::cross(const Vec3 &v) const {
	return Vec3(e_[1] * v.e_[2] - e_[2] * v.e_[1], e_[2] * v.e_[0] - e_[0] * v.e_[2],
		e_[0] * v.e_[1] - e_[1] * v.e_[0]);
}

// Check if vector is near zero (for floating point precision)
bool Vec3::near_zero() const {
	const auto s = 1e-8;
	return (std::fabs(e_[0]) < s) && (std::fabs(e_[1]) < s) && (std::fabs(e_[2]) < s);
}

// Vector addition
Vec3 operator+(const Vec3 &u, const Vec3 &v) {
	return Vec3(u.e_[0] + v.e_[0], u.e_[1] + v.e_[1], u.e_[2] + v.e_[2]);
}

// Vector subtraction
Vec3 operator-(const Vec3 &u, const Vec3 &v) {
	return Vec3(u.e_[0] - v.e_[0], u.e_[1] - v.e_[1], u.e_[2] - v.e_[2]);
}

// Component-wise multiplication
Vec3 operator*(const Vec3 &u, const Vec3 &v) {
	return Vec3(u.e_[0] * v.e_[0], u.e_[1] * v.e_[1], u.e_[2] * v.e_[2]);
}

// Scalar multiplication (scalar first)
Vec3 operator*(double t, const Vec3 &v) {
	return Vec3(t * v.e_[0], t * v.e_[1], t * v.e_[2]);
}

// Scalar multiplication (scalar second)
Vec3 operator*(const Vec3 &v, double t) {
	return t * v;
}

// Scalar division - returns invalid vector (NaN) if division by zero
Vec3 operator/(const Vec3 &v, double t) {
	if (t == 0.0) {
		return Vec3(NAN, NAN, NAN);
	}
	return (1 / t) * v;
}

// Reflection vector calculation
Vec3 reflect(const Vec3 &v, const Vec3 &n) {
	return v - 2 * v.dot(n) * n;
}

// Refraction vector calculation - returns invalid vector (NaN) if total
// internal reflection occurs
Vec3 refract(const Vec3 &uv, const Vec3 &n, double etai_over_etat) {
	auto cos_theta = std::fmin((-uv).dot(n), 1.0);
	Vec3 r_out_perp = etai_over_etat * (uv + cos_theta * n);
	Vec3 r_out_parallel = -std::sqrt(std::fabs(1.0 - r_out_perp.length_squared())) * n;

	// Check for total internal reflection
	if (std::isnan(r_out_parallel.length()) || std::isinf(r_out_parallel.length())) {
		return Vec3(NAN, NAN, NAN);
	}

	return r_out_perp + r_out_parallel;
}

} // namespace are
