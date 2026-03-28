#version 430 core

layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;
layout(location = 2) in vec2 a_texcoord;
layout(location = 3) in vec3 a_tangent;

out VS_OUT {
    vec3 frag_pos;
    vec3 normal;
    vec2 texcoord;
    vec3 tangent;
} vs_out;

uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_projection;

void main() {
    vec4 world_pos = u_model * vec4(a_position, 1.0);
    vs_out.frag_pos = world_pos.xyz;
    vs_out.normal = mat3(transpose(inverse(u_model))) * a_normal;
    vs_out.texcoord = a_texcoord;
    vs_out.tangent = mat3(u_model) * a_tangent;
    
    gl_Position = u_projection * u_view * world_pos;
}
