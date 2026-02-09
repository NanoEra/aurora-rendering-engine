#version 430 core

in vec3 v_world_position;
in vec3 v_world_normal;
in vec2 v_texcoord;
flat in uint v_triangle_id_base;

uniform vec3 u_albedo;
uniform float u_metallic;
uniform float u_roughness;

layout(location = 0) out vec3 g_position;
layout(location = 1) out vec3 g_normal;
layout(location = 2) out vec4 g_albedo_metallic;
layout(location = 3) out vec2 g_roughness_ao;
layout(location = 4) out uint g_primitive_id;

void main() {
    g_position = v_world_position;
    g_normal = normalize(v_world_normal);
    g_albedo_metallic = vec4(u_albedo, u_metallic);
    g_roughness_ao = vec2(u_roughness, 1.0);

    // Global primitive ID = mesh base + primitive id within draw call
    g_primitive_id = v_triangle_id_base + uint(gl_PrimitiveID);
}
