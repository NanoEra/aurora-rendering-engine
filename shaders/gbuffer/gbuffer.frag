#version 430 core

in VS_OUT {
    vec3 frag_pos;
    vec3 normal;
    vec2 texcoord;
    vec3 tangent;
} fs_in;

layout(location = 0) out vec4 g_position;
layout(location = 1) out vec2 g_normal;  // Octahedral encoded normal (RG32F)
layout(location = 2) out vec4 g_albedo;
layout(location = 3) out vec4 g_material;
layout(location = 4) out uint g_material_id;
layout(location = 5) out vec4 g_texcoord;
layout(location = 6) out vec4 g_tangent;

uniform vec3 u_albedo;
uniform float u_metallic;
uniform float u_roughness;
uniform float u_ior;
uniform vec3 u_emission;
uniform uint u_material_type;
uniform uint u_material_id;

uniform bool u_has_albedo_map;
uniform sampler2D u_albedo_map;

// Octahedral encode a unit vector to 2D coordinates in [0, 1] range
vec2 oct_encode(vec3 n) {
    float sum = abs(n.x) + abs(n.y) + abs(n.z);
    n /= sum;

    if (n.z < 0.0) {
        float x = n.x;
        float y = n.y;
        n.x = (1.0 - abs(y)) * (x >= 0.0 ? 1.0 : -1.0);
        n.y = (1.0 - abs(x)) * (y >= 0.0 ? 1.0 : -1.0);
    }

    return n.xy * 0.5 + 0.5;
}

void main() {
    g_position = vec4(fs_in.frag_pos, 1.0);

    vec3 n = normalize(fs_in.normal);
    g_normal = oct_encode(n);  // Encode normal as 2D coordinates

    vec3 albedo = u_albedo;
    if (u_has_albedo_map) {
        albedo *= texture(u_albedo_map, fs_in.texcoord).rgb;
    }
    g_albedo = vec4(albedo, 1.0);

    g_material = vec4(u_metallic, u_roughness, u_ior, float(u_material_type));
    g_material_id = u_material_id;
    
    // Store texcoord
    g_texcoord = vec4(fs_in.texcoord, 0.0, 0.0);
    
    // Store tangent (xyz = tangent, w = unused)
    g_tangent = vec4(fs_in.tangent, 0.0);
}
