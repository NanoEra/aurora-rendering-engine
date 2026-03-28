// Sampling utility functions

#ifndef SAMPLING_GLSL
#define SAMPLING_GLSL

// Cosine-weighted hemisphere sampling (avoids infinite loop)
vec3 random_in_unit_sphere(inout uint seed) {
    float z = 1.0 - 2.0 * random_float(seed);
    float r = sqrt(max(0.0, 1.0 - z * z));
    float phi = 2.0 * PI * random_float(seed);
    return vec3(r * cos(phi), r * sin(phi), z);
}

vec3 random_unit_vector(inout uint seed) {
    return normalize(random_in_unit_sphere(seed));
}

#endif // SAMPLING_GLSL
