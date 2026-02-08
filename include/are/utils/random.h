/**
 * @file random.h
 * @brief Random number generation utilities
 */

#ifndef ARE_INCLUDE_UTILS_RANDOM_H
#define ARE_INCLUDE_UTILS_RANDOM_H

#include <are/core/types.h>
#include <random>

namespace are {

/**
 * @class RandomGenerator
 * @brief Thread-safe random number generator
 *
 * Uses PCG (Permuted Congruential Generator) for high-quality random numbers.
 */
class RandomGenerator {
public:
	/**
	 * @brief Constructor with optional seed
	 * @param seed Random seed (0 = use random device)
	 */
	explicit RandomGenerator(uint64_t seed = 0);

	/**
	 * @brief Generate random float in [0, 1)
	 * @return Random float
	 */
	Real random_float();

	/**
	 * @brief Generate random float in [min, max)
	 * @param min Minimum value
	 * @param max Maximum value
	 * @return Random float
	 */
	Real random_float(Real min, Real max);

	/**
	 * @brief Generate random integer in [min, max]
	 * @param min Minimum value
	 * @param max Maximum value
	 * @return Random integer
	 */
	int random_int(int min, int max);

	/**
	 * @brief Generate random point in unit disk
	 * @return Random point (z = 0)
	 */
	Vec3 random_in_unit_disk();

	/**
	 * @brief Generate random point in unit sphere
	 * @return Random point
	 */
	Vec3 random_in_unit_sphere();

	/**
	 * @brief Generate random unit vector
	 * @return Random unit vector
	 */
	Vec3 random_unit_vector();

	/**
	 * @brief Generate random vector in hemisphere
	 * @param normal Hemisphere normal
	 * @return Random vector in hemisphere
	 */
	Vec3 random_in_hemisphere(const Vec3 &normal);

	/**
	 * @brief Generate random cosine-weighted direction
	 * @param normal Surface normal
	 * @return Random direction (cosine-weighted)
	 */
	Vec3 random_cosine_direction(const Vec3 &normal);

	/**
	 * @brief Set seed for reproducible results
	 * @param seed Random seed
	 */
	void set_seed(uint64_t seed);

private:
	std::mt19937_64 rng_; ///< Random number generator
	std::uniform_real_distribution<Real> dist_; ///< Uniform distribution [0, 1)
};

/**
 * @brief Get thread-local random generator
 * @return Reference to thread-local generator
 */
RandomGenerator &get_thread_random();

/**
 * @brief Generate random float in [0, 1) using thread-local generator
 * @return Random float
 */
inline Real random_float() {
	return get_thread_random().random_float();
}

/**
 * @brief Generate random float in [min, max) using thread-local generator
 * @param min Minimum value
 * @param max Maximum value
 * @return Random float
 */
inline Real random_float(Real min, Real max) {
	return get_thread_random().random_float(min, max);
}

} // namespace are

#endif // ARE_INCLUDE_UTILS_RANDOM_H
