// BVH traversal and ray-triangle intersection

#ifndef BVH_GLSL
#define BVH_GLSL

// Octahedral decode: 2D coordinates in [0, 1] range to unit vector
vec3 oct_decode(vec2 f) {
    f = f * 2.0 - 1.0;

    vec3 n = vec3(f.x, f.y, 1.0 - abs(f.x) - abs(f.y));
    float t = max(-n.z, 0.0);
    n.x += n.x >= 0.0 ? -t : t;
    n.y += n.y >= 0.0 ? -t : t;

    return normalize(n);
}

// Ray-AABB intersection
bool intersect_aabb(Ray ray, vec3 aabb_min, vec3 aabb_max, float t_max) {
    vec3 inv_d = 1.0 / ray.direction;
    vec3 t0 = (aabb_min - ray.origin) * inv_d;
    vec3 t1 = (aabb_max - ray.origin) * inv_d;

    vec3 tmin3 = min(t0, t1);
    vec3 tmax3 = max(t0, t1);

    float tmin = max(max(tmin3.x, tmin3.y), tmin3.z);
    float tmax2 = min(min(tmax3.x, tmax3.y), tmax3.z);

    return (tmax2 >= max(tmin, 0.0)) && (tmin <= t_max);
}

// Moller-Trumbore triangle intersection
bool intersect_triangle(Ray ray, TriangleGpu tri, inout HitInfo hit) {
    vec3 v0 = tri.v0_material.xyz;
    vec3 v1 = tri.v1.xyz;
    vec3 v2 = tri.v2.xyz;

    vec3 e1 = v1 - v0;
    vec3 e2 = v2 - v0;
    vec3 pvec = cross(ray.direction, e2);
    float det = dot(e1, pvec);

    if (abs(det) < EPSILON) return false;
    float inv_det = 1.0 / det;

    vec3 tvec = ray.origin - v0;
    float u = dot(tvec, pvec) * inv_det;
    if (u < 0.0 || u > 1.0) return false;

    vec3 qvec = cross(tvec, e1);
    float v = dot(ray.direction, qvec) * inv_det;
    if (v < 0.0 || u + v > 1.0) return false;

    float t = dot(e2, qvec) * inv_det;
    if (t < EPSILON || t >= hit.t) return false;

    float w = 1.0 - u - v;
    vec3 n0 = tri.n0.xyz;
    vec3 n1 = tri.n1.xyz;
    vec3 n2 = tri.n2.xyz;

    vec2 uv0 = tri.uv0_uv1.xy;
    vec2 uv1 = tri.uv0_uv1.zw;
    vec2 uv2 = tri.uv2.xy;

    vec3 t0 = tri.t0.xyz;
    vec3 t1 = tri.t1.xyz;
    vec3 t2 = normalize(cross(n0, t0));

    hit.hit = true;
    hit.t = t;
    hit.position = ray.origin + t * ray.direction;
    hit.normal = normalize(n0 * w + n1 * u + n2 * v);
    hit.texcoord = uv0 * w + uv1 * u + uv2 * v;
    hit.tangent = normalize(t0 * w + t1 * u + t2 * v);
    hit.material_id = as_uint(tri.v0_material.w);
    return true;
}

// BVH traversal (closest hit)
HitInfo trace_ray_bvh(Ray ray) {
    HitInfo hit;
    hit.hit = false;
    hit.t = MAX_FLOAT;

    if (!u_use_bvh || u_bvh_node_count == 0u) {
        return hit;
    }

    uint stack[64];
    int sp = 0;
    stack[sp++] = 0u;

    while (sp > 0) {
        uint node_idx = stack[--sp];
        if (node_idx >= u_bvh_node_count) continue;

        BVHNodeGpu node = bvh_nodes[node_idx];
        vec3 bmin = node.aabb_min_left_first.xyz;
        vec3 bmax = node.aabb_max_count.xyz;
        uint left_first = as_uint(node.aabb_min_left_first.w);
        uint count = as_uint(node.aabb_max_count.w);

        if (!intersect_aabb(ray, bmin, bmax, hit.t)) continue;

        if (count > 0u) {
            for (uint i = 0u; i < count; ++i) {
                TriangleGpu tri = bvh_tris[left_first + i];
                intersect_triangle(ray, tri, hit);
            }
        } else {
            uint left = left_first;
            uint right = left_first + 1u;
            if (sp < 63) stack[sp++] = right;
            if (sp < 63) stack[sp++] = left;
        }
    }

    return hit;
}

// Any-hit BVH for shadow ray
bool trace_any_bvh(Ray ray, float t_max) {
    if (!u_use_bvh || u_bvh_node_count == 0u) return false;

    uint stack[64];
    int sp = 0;
    stack[sp++] = 0u;

    HitInfo hit;
    hit.hit = false;
    hit.t = t_max;

    while (sp > 0) {
        uint node_idx = stack[--sp];
        if (node_idx >= u_bvh_node_count) continue;

        BVHNodeGpu node = bvh_nodes[node_idx];
        vec3 bmin = node.aabb_min_left_first.xyz;
        vec3 bmax = node.aabb_max_count.xyz;
        uint left_first = as_uint(node.aabb_min_left_first.w);
        uint count = as_uint(node.aabb_max_count.w);

        if (!intersect_aabb(ray, bmin, bmax, hit.t)) continue;

        if (count > 0u) {
            for (uint i = 0u; i < count; ++i) {
                TriangleGpu tri = bvh_tris[left_first + i];
                if (intersect_triangle(ray, tri, hit)) return true;
            }
        } else {
            uint left = left_first;
            uint right = left_first + 1u;
            if (sp < 63) stack[sp++] = right;
            if (sp < 63) stack[sp++] = left;
        }
    }

    return false;
}

// Read primary hit from G-Buffer
HitInfo trace_primary_gbuffer(Ray ray, ivec2 pixel_coords) {
    HitInfo hit;
    hit.hit = false;
    hit.t = MAX_FLOAT;
    hit.position = vec3(0.0);
    hit.normal = vec3(0.0, 1.0, 0.0);
    hit.texcoord = vec2(0.0);
    hit.tangent = vec3(0.0);
    hit.material_id = 0u;
    hit.material_type = 0;

    vec4 pos = imageLoad(g_position, pixel_coords);
    if (pos.w <= 0.5) {
        return hit;
    }

    vec3 p = pos.xyz;
    
    // Decode octahedral normal from RG32F
    vec2 oct_n = imageLoad(g_normal, pixel_coords).xy;
    vec3 n = oct_decode(oct_n);
    
    uint mid = imageLoad(g_material_id, pixel_coords).r;
    vec4 mat = imageLoad(g_material, pixel_coords);
    int mtype = int(mat.w);
    vec4 texcoord_tangent = imageLoad(g_texcoord, pixel_coords);
    vec2 texcoord = texcoord_tangent.xy;
    vec4 tangent_data = imageLoad(g_tangent, pixel_coords);
    vec3 tangent = tangent_data.xyz;

    hit.hit = true;
    hit.position = p;
    hit.normal = n;
    hit.texcoord = texcoord;
    hit.tangent = tangent;
    hit.material_id = mid;
    hit.material_type = mtype;
    hit.t = length(p - ray.origin);

    return hit;
}

#endif // BVH_GLSL
