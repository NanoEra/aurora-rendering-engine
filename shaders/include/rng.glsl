// PCG Random Number Generator

#ifndef RNG_GLSL
#define RNG_GLSL

uint pcg_hash(uint seed) {
    uint state = seed * 747796405u + 2891336453u;
    uint word = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
    return (word >> 22u) ^ word;
}

// Improved seed initialization with spatial-temporal decorrelation
uint init_seed(ivec2 pixel_coords, ivec2 image_size, uint frame_count) {
    uint pixel_index = uint(pixel_coords.x) + uint(pixel_coords.y) * uint(image_size.x);
    
    // Spatial hash to decorrelate neighboring pixels
    uint spatial = pcg_hash(pixel_index);
    
    // Temporal hash with pixel-dependent multiplier to avoid frame correlation
    uint temporal = frame_count * (spatial | 1u);  // OR with 1 to ensure non-zero multiplier
    
    return pcg_hash(spatial + temporal);
}

float random_float(inout uint seed) {
    seed = pcg_hash(seed);
    return float(seed) / 4294967296.0;
}

vec3 random_vec3(inout uint seed) {
    return vec3(random_float(seed), random_float(seed), random_float(seed));
}

#endif // RNG_GLSL
