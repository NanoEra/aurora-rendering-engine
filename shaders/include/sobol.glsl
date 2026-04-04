// Sobol Low-Discrepancy Sequence
// Provides O(1/n) convergence vs O(1/sqrt(n)) for white noise

#ifndef SOBOL_GLSL
#define SOBOL_GLSL

// Sobol direction numbers for first 8 dimensions (32-bit)
// Each dimension has 32 direction numbers (one per bit)
// Using the standard Sobol initialization with primitive polynomials

// Dimension 0: primitive polynomial x (degree 1)
const uint sobol_dirs_0[32] = uint[32](
    0x80000000u, 0x40000000u, 0x20000000u, 0x10000000u,
    0x08000000u, 0x04000000u, 0x02000000u, 0x01000000u,
    0x00800000u, 0x00400000u, 0x00200000u, 0x00100000u,
    0x00080000u, 0x00040000u, 0x00020000u, 0x00010000u,
    0x00008000u, 0x00004000u, 0x00002000u, 0x00001000u,
    0x00000800u, 0x00000400u, 0x00000200u, 0x00000100u,
    0x00000080u, 0x00000040u, 0x00000020u, 0x00000010u,
    0x00000008u, 0x00000004u, 0x00000002u, 0x00000001u
);

// Dimension 1: primitive polynomial 1+x (degree 2)
const uint sobol_dirs_1[32] = uint[32](
    0x80000000u, 0xc0000000u, 0xa0000000u, 0xf0000000u,
    0x88000000u, 0xcc000000u, 0xaa000000u, 0xff000000u,
    0x80800000u, 0xc0c00000u, 0xa0a00000u, 0xf0f00000u,
    0x88880000u, 0xcccc0000u, 0xaaaa0000u, 0xffff0000u,
    0x80008000u, 0xc000c000u, 0xa000a000u, 0xf000f000u,
    0x88008800u, 0xcc00cc00u, 0xaa00aa00u, 0xff00ff00u,
    0x80808080u, 0xc0c0c0c0u, 0xa0a0a0a0u, 0xf0f0f0f0u,
    0x88888888u, 0xccccccccu, 0xaaaaaaaau, 0xffffffffu
);

// Dimension 2: primitive polynomial 1+x+x^2 (degree 3)
const uint sobol_dirs_2[32] = uint[32](
    0x80000000u, 0xc0000000u, 0x60000000u, 0x90000000u,
    0xe8000000u, 0x5c000000u, 0x86000000u, 0xc9000000u,
    0x6e800000u, 0x95c00000u, 0xe8600000u, 0x5c900000u,
    0x86e80000u, 0xc95c0000u, 0x6e860000u, 0x95c90000u,
    0xe86e8000u, 0x5c95c000u, 0x86e86000u, 0xc95c9000u,
    0x6e86e800u, 0x95c95c00u, 0xe86e8600u, 0x5c95c900u,
    0x86e86e80u, 0xc95c95c0u, 0x6e86e860u, 0x95c95c90u,
    0xe86e86e8u, 0x5c95c95cu, 0x86e86e86u, 0xc95c95c9u
);

// Dimension 3: primitive polynomial 1+x+x^3 (degree 3)
const uint sobol_dirs_3[32] = uint[32](
    0x80000000u, 0x40000000u, 0x20000000u, 0xd0000000u,
    0xf8000000u, 0x6c000000u, 0x9a000000u, 0xc1000000u,
    0x78800000u, 0xb4c00000u, 0x52600000u, 0xa9100000u,
    0xd0880000u, 0xe84c0000u, 0x6ca60000u, 0x9a110000u,
    0xc1088000u, 0x7884c000u, 0xb4c26000u, 0x52691000u,
    0xa9108800u, 0xd0884c00u, 0xe84c2600u, 0x6ca69100u,
    0x9a110880u, 0xc10884c0u, 0x7884c260u, 0xb4c26910u,
    0x52691088u, 0xa910884cu, 0xd0884c26u, 0xe84c2691u
);

// Dimension 4: primitive polynomial 1+x^2+x^3 (degree 3)
const uint sobol_dirs_4[32] = uint[32](
    0x80000000u, 0xc0000000u, 0xa0000000u, 0x50000000u,
    0xb8000000u, 0x6c000000u, 0x86000000u, 0x43000000u,
    0xa1800000u, 0x5ec00000u, 0xb0600000u, 0x6c100000u,
    0x86a80000u, 0x435c0000u, 0xa1860000u, 0x5ec10000u,
    0xb06a8000u, 0x6c15c000u, 0x86a86000u, 0x435c1000u,
    0xa186a800u, 0x5ec15c00u, 0xb06a8600u, 0x6c15c100u,
    0x86a86a80u, 0x435c15c0u, 0xa186a860u, 0x5ec15c10u,
    0xb06a86a8u, 0x6c15c15cu, 0x86a86a86u, 0x435c15c1u
);

// Dimension 5: primitive polynomial 1+x+x^2+x^4 (degree 4)
const uint sobol_dirs_5[32] = uint[32](
    0x80000000u, 0x40000000u, 0x20000000u, 0x10000000u,
    0xf8000000u, 0xdc000000u, 0x6a000000u, 0x35000000u,
    0x1a800000u, 0x8dc00000u, 0x46a00000u, 0x23500000u,
    0x11a80000u, 0xf8dc0000u, 0xdc6a0000u, 0x6a350000u,
    0x351a8000u, 0x1a8dc000u, 0x8dc6a000u, 0x46a35000u,
    0x2351a800u, 0x11a8dc00u, 0xf8dc6a00u, 0xdc6a3500u,
    0x6a351a80u, 0x351a8dc0u, 0x1a8dc6a0u, 0x8dc6a350u,
    0x46a351a8u, 0x2351a8dcu, 0x11a8dc6au, 0xf8dc6a35u
);

// Dimension 6: primitive polynomial 1+x+x^3+x^4 (degree 4)
const uint sobol_dirs_6[32] = uint[32](
    0x80000000u, 0xc0000000u, 0xe0000000u, 0x70000000u,
    0x38000000u, 0x9c000000u, 0x4e000000u, 0xa7000000u,
    0xd3800000u, 0x69c00000u, 0xb4e00000u, 0x5a700000u,
    0x2d380000u, 0x169c0000u, 0x8b4e0000u, 0x45a70000u,
    0xa2d38000u, 0xd169c000u, 0x68b4e000u, 0xb45a7000u,
    0x5a2d3800u, 0x2d169c00u, 0x168b4e00u, 0x8b45a700u,
    0x45a2d380u, 0xa2d169c0u, 0xd168b4e0u, 0x68b45a70u,
    0xb45a2d38u, 0x5a2d169cu, 0x2d168b4eu, 0x168b45a7u
);

// Dimension 7: primitive polynomial 1+x^2+x^4 (degree 4)
const uint sobol_dirs_7[32] = uint[32](
    0x80000000u, 0xc0000000u, 0xa0000000u, 0x50000000u,
    0x28000000u, 0x14000000u, 0x8a000000u, 0x45000000u,
    0xa2800000u, 0xd1400000u, 0xe8a00000u, 0x74500000u,
    0x3a280000u, 0x1d140000u, 0x8e8a0000u, 0x47450000u,
    0x23a28000u, 0x11d14000u, 0x88e8a000u, 0x44745000u,
    0xa23a2800u, 0xd11d1400u, 0xe88e8a00u, 0x74474500u,
    0x3a23a280u, 0x1d11d140u, 0x8e88e8a0u, 0x47447450u,
    0x23a23a28u, 0x11d11d14u, 0x88e88e8au, 0x44744745u
);

// Access direction numbers by dimension
uint sobol_direction(uint dimension, uint bit) {
    if (dimension == 0u) return sobol_dirs_0[bit];
    if (dimension == 1u) return sobol_dirs_1[bit];
    if (dimension == 2u) return sobol_dirs_2[bit];
    if (dimension == 3u) return sobol_dirs_3[bit];
    if (dimension == 4u) return sobol_dirs_4[bit];
    if (dimension == 5u) return sobol_dirs_5[bit];
    if (dimension == 6u) return sobol_dirs_6[bit];
    if (dimension == 7u) return sobol_dirs_7[bit];
    return sobol_dirs_0[bit];  // Fallback
}

// Owen scrambling for decorrelation
uint owen_scramble(uint value, uint seed) {
    // Simple hash-based Owen scrambling
    uint s = seed ^ value;
    s ^= s >> 16;
    s *= 0x45d9f3bu;
    s ^= s >> 16;
    s *= 0x45d9f3bu;
    s ^= s >> 16;
    return s;
}

// Generate Sobol value for given index and dimension
// index: sample index (0, 1, 2, ...)
// dimension: which dimension (0, 1, 2, ...)
// scramble_seed: seed for Owen scrambling
float sobol_get(uint index, uint dimension, uint scramble_seed) {
    uint result = 0u;
    uint i = index;
    
    // Gray code iteration
    for (uint bit = 0u; bit < 32u; bit++) {
        if ((i & 1u) != 0u) {
            result ^= sobol_direction(dimension, bit);
        }
        i >>= 1u;
    }
    
    // Apply Owen scrambling
    result = owen_scramble(result, scramble_seed);
    
    return float(result) / 4294967296.0;
}

#endif // SOBOL_GLSL
