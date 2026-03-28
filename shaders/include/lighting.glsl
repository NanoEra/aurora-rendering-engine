// Direct lighting with shadow rays

#ifndef LIGHTING_GLSL
#define LIGHTING_GLSL

vec3 eval_direct_lighting(inout HitInfo hit, Material mat, inout uint seed) {
    if (u_light_count == 0u) return vec3(0.0);

    uint light_idx = uint(random_float(seed) * float(u_light_count)) % u_light_count;
    Light light = lights[light_idx];

    vec3 L;
    float dist = MAX_FLOAT;
    vec3 radiance = vec3(0.0);

    if (light.type == LIGHT_POINT) {
        vec3 to_light = light.position - hit.position;
        dist = length(to_light);
        if (dist > light.range) return vec3(0.0);
        L = to_light / dist;

        float atten = 1.0 / max(dist * dist, 0.01);
        radiance = light.color * light.intensity * atten;
    } else if (light.type == LIGHT_DIRECTIONAL) {
        L = normalize(-light.direction);
        radiance = light.color * light.intensity;
    } else {
        return vec3(0.0);
    }

    float n_dot_l = max(dot(hit.normal, L), 0.0);
    if (n_dot_l <= 0.0) return vec3(0.0);

    Ray shadow_ray;
    shadow_ray.origin = hit.position + hit.normal * EPSILON;
    shadow_ray.direction = L;

    float t_max = (light.type == LIGHT_POINT) ? (dist - EPSILON) : MAX_FLOAT;
    if (trace_any_bvh(shadow_ray, t_max)) return vec3(0.0);

    float pdf_light = 1.0 / float(u_light_count);
    vec3 brdf = mat.albedo * INV_PI;
    return brdf * radiance * n_dot_l * mat.ao / max(pdf_light, EPSILON);
}

vec3 environment_color(vec3 dir) {
    return vec3(0.1, 0.1, 0.15);
}

#endif // LIGHTING_GLSL
