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

// Build orthonormal basis from normal vector
mat3 build_onb(vec3 N) {
    vec3 up = (abs(N.y) < 0.999) ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0);
    vec3 T = normalize(cross(up, N));
    vec3 B = cross(N, T);
    return mat3(T, B, N);
}

// GGX importance sampling: sample microfacet normal (half vector)
// Returns sampled half vector in world space
vec3 sample_ggx_half_vector(float roughness, vec3 N, inout uint seed) {
    float a = roughness * roughness;
    float a2 = a * a;

    float u1 = random_float(seed);
    float u2 = random_float(seed);

	// Clamp to avoid numerical issues at boundaries
	u1 = clamp(u1, 0.001, 0.999);

    // Spherical coordinates from GGX distribution
    float cos_theta = sqrt((1.0 - u1) / ((a2 - 1.0) * u1 + 1.0));
	cos_theta = clamp(cos_theta, 0.0, 1.0);
    float sin_theta = sqrt(max(0.0, 1.0 - cos_theta * cos_theta));
    float phi = 2.0 * PI * u2;

    // Convert to Cartesian in tangent space
    vec3 H_tangent = vec3(sin_theta * cos(phi), sin_theta * sin(phi), cos_theta);

    // Transform to world space
    mat3 onb = build_onb(N);
    return normalize(onb * H_tangent);
}

// GGX importance sampling PDF
float ggx_pdf(float NdotH, float roughness) {
    float a = roughness * roughness;
    float a2 = a * a;
    float denom = NdotH * NdotH * (a2 - 1.0) + 1.0;
    float D = a2 / (PI * denom * denom);
    return D * NdotH;
}

#endif // SAMPLING_GLSL
