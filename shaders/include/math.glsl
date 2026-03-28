// Math utility functions

#ifndef MATH_GLSL
#define MATH_GLSL

bool near_zero(vec3 v) {
    return (abs(v.x) < EPSILON) && (abs(v.y) < EPSILON) && (abs(v.z) < EPSILON);
}

vec3 reflect_vector(vec3 v, vec3 n) {
    return v - 2.0 * dot(v, n) * n;
}

vec3 refract_vector(vec3 uv, vec3 n, float etai_over_etat) {
    float cos_theta = min(dot(-uv, n), 1.0);
    vec3 r_out_perp = etai_over_etat * (uv + cos_theta * n);
    vec3 r_out_parallel = -sqrt(abs(1.0 - dot(r_out_perp, r_out_perp))) * n;
    return r_out_perp + r_out_parallel;
}

uint as_uint(float f) { return floatBitsToUint(f); }
float as_float(uint u) { return uintBitsToFloat(u); }

#endif // MATH_GLSL
