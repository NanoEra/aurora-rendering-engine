// ACES Filmic Tone Mapping — shared between ray tracing and upscale passes.
// Converts HDR linear radiance to LDR display colour [0, 1].
vec3 aces_tonemap(vec3 x) {
	float a = 2.51, b = 0.03, c = 2.43, d = 0.59, e = 0.14;
	return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}
