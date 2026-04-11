// Material handling and PBR scattering

#ifndef MATERIAL_GLSL
#define MATERIAL_GLSL

// Helper function to sample texture from array by index
vec4 sample_texture_array(int slot, int index, vec2 uv) {
    if (index <= 0) return vec4(1.0);
    
    if (slot == 0) return texture(u_texture_albedo_array, vec3(uv, float(index - 1)));
    if (slot == 1) return texture(u_texture_normal_array, vec3(uv, float(index - 1)));
    if (slot == 2) return texture(u_texture_metallic_array, vec3(uv, float(index - 1)));
    if (slot == 3) return texture(u_texture_roughness_array, vec3(uv, float(index - 1)));
    if (slot == 4) return texture(u_texture_ao_array, vec3(uv, float(index - 1)));
    if (slot == 5) return texture(u_texture_emission_array, vec3(uv, float(index - 1)));
    
    return vec4(1.0);
}

// Apply normal map in world space
vec3 apply_normal_map(vec3 normal, vec2 texcoord, vec3 tangent, uint normal_handle) {
    if (normal_handle == 0 || !u_enable_textures) return normal;
    
    vec3 T = normalize(tangent - normal * dot(tangent, normal));
    vec3 B = cross(normal, T);
    mat3 TBN = mat3(T, B, normal);
    
    vec3 map_n = sample_texture_array(1, int(normal_handle), texcoord).xyz * 2.0 - 1.0;
    return normalize(TBN * map_n);
}

// Apply material textures to get final PBR values
void apply_material_textures(inout Material mat, inout vec3 normal, vec2 texcoord, vec3 tangent) {
    if (!u_enable_textures) return;
    
    if (mat.texture_handles[0] != 0) {
        mat.albedo = sample_texture_array(0, int(mat.texture_handles[0]), texcoord).rgb;
    }
    
    if (mat.texture_handles[1] != 0) {
        normal = apply_normal_map(normal, texcoord, tangent, mat.texture_handles[1]);
    }
    
    if (mat.texture_handles[2] != 0) {
        mat.metallic = sample_texture_array(2, int(mat.texture_handles[2]), texcoord).r;
    }
    
    if (mat.texture_handles[3] != 0) {
        mat.roughness = sample_texture_array(3, int(mat.texture_handles[3]), texcoord).r;
    }
    
    if (mat.texture_handles[4] != 0) {
        mat.ao = sample_texture_array(4, int(mat.texture_handles[4]), texcoord).r;
    }
    
    if (mat.texture_handles[5] != 0) {
        mat.emission = sample_texture_array(5, int(mat.texture_handles[5]), texcoord).rgb;
    }
}

// Fresnel functions
vec3 fresnel_schlick(float cos_theta, vec3 f0) {
    return f0 + (1.0 - f0) * pow(1.0 - cos_theta, 5.0);
}

float fresnel_dielectric(float cos_theta, float ior) {
    float r0 = (1.0 - ior) / (1.0 + ior);
    r0 = r0 * r0;
    return r0 + (1.0 - r0) * pow(1.0 - cos_theta, 5.0);
}

// GGX/Trowbridge-Reitz normal distribution function
float distribution_ggx(float NdotH, float roughness) {
    float a = roughness * roughness;
    float a2 = a * a;
    float d = NdotH * NdotH * (a2 - 1.0) + 1.0;
    return a2 / (PI * d * d);
}

// Scatter functions
ScatterResult scatter_diffuse(Ray ray_in, HitInfo hit, Material mat, inout uint seed) {
    ScatterResult r;
    r.scattered = true;
    r.attenuation = mat.albedo;

    // Use cosine-weighted importance sampling for Lambertian BRDF
    vec3 dir = sample_cosine_weighted(hit.normal, seed);
    if (near_zero(dir)) dir = hit.normal;

    r.scattered_ray.origin = hit.position + hit.normal * EPSILON;
    r.scattered_ray.direction = dir;
    return r;
}

ScatterResult scatter_metal(Ray ray_in, HitInfo hit, Material mat, inout uint seed) {
    ScatterResult r;

    vec3 V = normalize(-ray_in.direction);
    vec3 N = hit.normal;

    // Clamp roughness to avoid division by zero
    float roughness = max(mat.roughness, 0.04);

    // Sample microfacet normal using GGX importance sampling
    vec3 H = sample_ggx_half_vector(roughness, N, seed);

    // Reflect view direction around half vector
    vec3 L = reflect(-V, H);

    // Check if reflected direction is above surface
    float NdotL = dot(N, L);
    if (NdotL <= 0.0) {
        r.scattered = false;
        r.attenuation = vec3(0.0);
        return r;
    }

	float NdotV = max(dot(N, V), 0.001);
	float HdotV = max(dot(H, V), 0.001);

	// Fresnel term (using albedo as F0 for metals)
	vec3 F = fresnel_schlick(HdotV, mat.albedo);

	// With proper GGX importance sampling of H, the BRDF contribution
	// simplifies to just the Fresnel term.
	// The D and geometry terms are canceled by the PDF.
	r.attenuation = F;

    r.scattered = true;
    r.scattered_ray.origin = hit.position + N * EPSILON;
    r.scattered_ray.direction = L;
    return r;
}

ScatterResult scatter_dielectric(Ray ray_in, HitInfo hit, Material mat, inout uint seed) {
    ScatterResult r;
    r.scattered = true;
    r.attenuation = vec3(1.0);

    vec3 unit_dir = normalize(ray_in.direction);
    float cos_theta = dot(-unit_dir, hit.normal);
    float sin_theta = sqrt(max(0.0, 1.0 - cos_theta * cos_theta));
    
    bool entering = cos_theta > 0.0;
    float eta = entering ? (1.0 / mat.ior) : mat.ior;
    vec3 normal = entering ? hit.normal : -hit.normal;
    
    float sin_theta_t = eta * sin_theta;
    bool total_internal_reflection = sin_theta_t >= 1.0;
    
    float f0 = pow((1.0 - mat.ior) / (1.0 + mat.ior), 2.0);
    float f = f0 + (1.0 - f0) * pow(1.0 - abs(cos_theta), 5.0);
    
    vec3 dir;
    if (total_internal_reflection || random_float(seed) < f) {
        dir = reflect_vector(unit_dir, normal);
    } else {
        dir = refract_vector(unit_dir, normal, eta);
    }

    r.scattered_ray.origin = hit.position + dir * EPSILON;
    r.scattered_ray.direction = dir;
    return r;
}

ScatterResult scatter_ray(Ray ray_in, HitInfo hit, Material mat, inout uint seed) {
    if (mat.type == MATERIAL_DIFFUSE) return scatter_diffuse(ray_in, hit, mat, seed);
    if (mat.type == MATERIAL_METAL) return scatter_metal(ray_in, hit, mat, seed);
    if (mat.type == MATERIAL_DIELECTRIC) return scatter_dielectric(ray_in, hit, mat, seed);

    ScatterResult r;
    r.scattered = false;
    r.attenuation = vec3(0.0);
    return r;
}

// Fetch material with fallback
Material fetch_material(uint material_id) {
    uint cnt = uint(materials.length());
    if (material_id < cnt) return materials[material_id];

    Material m;
    m.albedo = vec3(0.5);
    m.metallic = 0.0;
    m.emission = vec3(0.0);
    m.roughness = 0.5;
    m.type = MATERIAL_DIFFUSE;
    m.ior = 1.5;
    m.ao = 1.0;
    return m;
}

#endif // MATERIAL_GLSL
