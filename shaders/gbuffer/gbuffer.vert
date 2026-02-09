#version 430 core

layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;
layout(location = 2) in vec2 a_texcoord;
layout(location = 3) in vec3 a_tangent;

uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_projection;
uniform mat3 u_normal_matrix;

// NOTE: We store it as int uniform in C++ for simplicity; GLSL expects uint.
uniform uint u_triangle_id_base;

out vec3 v_world_position;
out vec3 v_world_normal;
out vec2 v_texcoord;
flat out uint v_triangle_id_base;

void main() {
    vec4 world_pos = u_model * vec4(a_position, 1.0);
    v_world_position = world_pos.xyz;
    v_world_normal = normalize(u_normal_matrix * a_normal);
    v_texcoord = a_texcoord;
    v_triangle_id_base = u_triangle_id_base;

    gl_Position = u_projection * u_view * world_pos;
}
