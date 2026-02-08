#version 430 core

// Inputs from vertex shader
in vec3 v_world_position;
in vec3 v_world_normal;
in vec2 v_texcoord;
in vec3 v_world_tangent;

// Material uniforms
uniform vec3 u_albedo;
uniform float u_metallic;
uniform float u_roughness;

// G-Buffer outputs
layout(location = 0) out vec3 g_position;
layout(location = 1) out vec3 g_normal;
layout(location = 2) out vec4 g_albedo_metallic;
layout(location = 3) out vec2 g_roughness_ao;

void main() {
    // Output world position
    g_position = v_world_position;
    
    // Output normalized world normal
    g_normal = normalize(v_world_normal);

    // Output albedo (RGB) and metallic (A)
    g_albedo_metallic = vec4(u_albedo, u_metallic);
    
    // Output roughness (R) and ambient occlusion (G)
    // AO is set to 1.0 by default (no occlusion)
    g_roughness_ao = vec2(u_roughness, 1.0);
}
