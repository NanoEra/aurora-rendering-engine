#version 430 core

// Material types
const uint MATERIAL_DIFFUSE = 0u;
const uint MATERIAL_METAL = 1u;
const uint MATERIAL_DIELECTRIC = 2u;
const uint MATERIAL_EMISSIVE = 3u;

in VS_OUT {
    vec3 frag_pos;
    vec3 normal;
    vec2 texcoord;
    vec3 tangent;
} fs_in;

layout(location = 0) out vec4 g_position;
layout(location = 1) out vec4 g_normal;
layout(location = 2) out vec4 g_albedo;
layout(location = 3) out vec4 g_material;

// Material uniforms
uniform vec3 u_albedo;
uniform float u_metallic;
uniform float u_roughness;
uniform float u_ior;
uniform vec3 u_emission;
uniform uint u_material_type;

uniform bool u_has_albedo_map;
uniform sampler2D u_albedo_map;

void main() {
    // Position
    g_position = vec4(fs_in.frag_pos, 1.0);
    
    // Normal
    vec3 normal = normalize(fs_in.normal);
    g_normal = vec4(normal, 0.0);
    
    // Albedo
    vec3 albedo = u_albedo;
    if (u_has_albedo_map) {
        albedo *= texture(u_albedo_map, fs_in.texcoord).rgb;
    }
    g_albedo = vec4(albedo, 1.0);
    
    // Material properties
    g_material = vec4(u_metallic, u_roughness, u_ior, float(u_material_type));
}
