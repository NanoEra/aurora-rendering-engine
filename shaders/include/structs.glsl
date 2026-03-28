// Data structures for ray tracing

#ifndef STRUCTS_GLSL
#define STRUCTS_GLSL

struct Material {
    vec3 albedo;
    vec3 emission;
    float metallic;
    float roughness;
    int type;
    float ior;
    float ao;
    float padding1;
    uint texture_handles[6];
};

struct Light {
    vec3 position;
    int type;
    vec3 direction;
    float intensity;
    vec3 color;
    float range;
    vec2 spot_angles;
    vec2 padding;
};

struct Ray {
    vec3 origin;
    vec3 direction;
};

struct HitInfo {
    bool hit;
    float t;
    vec3 position;
    vec3 normal;
    vec2 texcoord;
    vec3 tangent;
    uint material_id;
    int material_type;
};

struct ScatterResult {
    bool scattered;
    vec3 attenuation;
    Ray scattered_ray;
};

struct BVHNodeGpu {
    vec4 aabb_min_left_first;
    vec4 aabb_max_count;
};

struct TriangleGpu {
    vec4 v0_material;
    vec4 v1;
    vec4 v2;
    vec4 n0;
    vec4 n1;
    vec4 n2;
    vec4 uv0_uv1;
    vec4 uv2;
    vec4 t0;
    vec4 t1;
};

#endif // STRUCTS_GLSL
