// PCG Random Number Generator

#ifndef RNG_GLSL
#define RNG_GLSL

uint pcg_hash(uint seed) {
    uint state = seed * 747796405u + 2891336453u;
    uint word = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
    return (word >> 22u) ^ word;
}

float random_float(inout uint seed) {
    seed = pcg_hash(seed);
    return float(seed) / 4294967296.0;
}

vec3 random_vec3(inout uint seed) {
    return vec3(random_float(seed), random_float(seed), random_float(seed));
}

#endif // RNG_GLSL
