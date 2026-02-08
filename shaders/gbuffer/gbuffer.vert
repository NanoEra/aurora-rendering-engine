#version 430 core

// Vertex attributes
layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;
layout(location = 2) in vec2 a_texcoord;
layout(location = 3) in vec3 a_tangent;

// Uniforms
uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_projection;
uniform mat3 u_normal_matrix;

// Outputs to fragment shader
out vec3 v_world_position;
out vec3 v_world_normal;
out vec2 v_texcoord;
out vec3 v_world_tangent;

void main() {
    // Transform position to world space
    vec4 world_pos = u_model * vec4(a_position, 1.0);
    v_world_position = world_pos.xyz;
    
    // Transform normal and tangent to world space
    v_world_normal = normalize(u_normal_matrix * a_normal);
    v_world_tangent = normalize(u_normal_matrix * a_tangent);
    
    // Pass through texture coordinates
    v_texcoord = a_texcoord;
    
    // Transform to clip space
    gl_Position = u_projection * u_view * world_pos;
}
