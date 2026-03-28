// Common constants and definitions for ray tracing

#ifndef COMMON_GLSL
#define COMMON_GLSL

// Mathematical constants
#define PI 3.14159265359
#define INV_PI 0.31830988618
#define EPSILON 1e-4
#define MAX_FLOAT 3.402823466e38
#define RR_THRESHOLD 0.1

// Material types
#define MATERIAL_DIFFUSE 0
#define MATERIAL_METAL 1
#define MATERIAL_DIELECTRIC 2
#define MATERIAL_EMISSIVE 3

// Light types
#define LIGHT_DIRECTIONAL 0
#define LIGHT_POINT 1
#define LIGHT_SPOT 2

// Texture slots
#define TEXTURE_SLOT_ALBEDO 0
#define TEXTURE_SLOT_NORMAL 1
#define TEXTURE_SLOT_METALLIC 2
#define TEXTURE_SLOT_ROUGHNESS 3
#define TEXTURE_SLOT_AO 4
#define TEXTURE_SLOT_EMISSION 5

#endif // COMMON_GLSL
