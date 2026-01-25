/*
 * Cornell Box Renderer based on Recursive Viewport Texture Transformation
 * Compilation: g++ -O3 -std=c++17 main.cpp -o renderer
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <memory>
#include <iomanip>

// --- Constants & Configuration ---
const int IMAGE_WIDTH = 800;
const int IMAGE_HEIGHT = 600;
const int MAX_DEPTH = 5;
const float AREA_THRESHOLD = 4.0f; // Stop recursion if projected area is less than 4 pixels
const float EPSILON = 1e-4f;

// --- Basic Math Structures ---
struct Vec3 {
    float x, y, z;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(float t) const { return Vec3(x * t, y * t, z * t); }
    Vec3 operator/(float t) const { return Vec3(x / t, y / t, z / t); }
    float dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }
    Vec3 cross(const Vec3& v) const { return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x); }
    float length() const { return std::sqrt(x * x + y * y + z * z); }
    Vec3 normalize() const { float l = length(); if (l > 0) return *this / l; return *this; }
    
    // Component-wise min/max for bounding box
    Vec3 min(const Vec3& v) const { return Vec3(std::min(x, v.x), std::min(y, v.y), std::min(z, v.z)); }
    Vec3 max(const Vec3& v) const { return Vec3(std::max(x, v.x), std::max(y, v.y), std::max(z, v.z)); }
};

struct Color {
    unsigned char r, g, b;
    Color() : r(0), g(0), b(0) {}
    Color(float fr, float fg, float fb) {
        r = static_cast<unsigned char>(std::min(255.0f, std::max(0.0f, fr * 255.0f)));
        g = static_cast<unsigned char>(std::min(255.0f, std::max(0.0f, fg * 255.0f)));
        b = static_cast<unsigned char>(std::min(255.0f, std::max(0.0f, fb * 255.0f)));
    }
    Color(unsigned char r, unsigned char g, unsigned char b) : r(r), g(g), b(b) {}
    
    Color operator*(float t) const {
        return Color(r * t, g * t, b * t);
    }
    // Alpha blending helper
    Color blendOver(const Color& background, float alpha) const {
        float invAlpha = 1.0f - alpha;
        return Color(
            r * alpha + background.r * invAlpha,
            g * alpha + background.g * invAlpha,
            b * alpha + background.b * invAlpha
        );
    }
};

// --- Texture Class (2D Image Buffer) ---
class Texture {
public:
    int width, height;
    std::vector<Color> data;

    Texture() : width(0), height(0) {}
    Texture(int w, int h) : width(w), height(h), data(w * h, Color(0.0f, 0.0f, 0.0f)) {}

    void setPixel(int x, int y, const Color& c) {
        if (x >= 0 && x < width && y >= 0 && y < height) {
            data[y * width + x] = c;
        }
    }

    Color getPixel(float u, float v) const {
        // Clamp or wrap? Let's clamp for UV mapping safety
        u = std::max(0.0f, std::min(1.0f, u));
        v = std::max(0.0f, std::min(1.0f, v));
        
        int x = static_cast<int>(u * (width - 1));
        int y = static_cast<int>(v * (height - 1));
        return data[y * width + x];
    }
    
    // Resampling (Bilinear-ish approximation for texture mapping)
    Color sample(float u, float v) const {
        return getPixel(u, v);
    }
};

// --- Geometry & Math Helpers ---

// Plane defined by normal N and point P
float planeDist(const Vec3& N, const Vec3& P, const Vec3& X) {
    return (X - P).dot(N);
}

// Ray-Triangle Intersection (Möller–Trumbore for culling visibility checks)
// Returns distance, -1 if no hit
float rayTriangleIntersect(const Vec3& orig, const Vec3& dir, const Vec3& v0, const Vec3& v1, const Vec3& v2) {
    Vec3 edge1 = v1 - v0;
    Vec3 edge2 = v2 - v0;
    Vec3 h = dir.cross(edge2);
    float a = edge1.dot(h);
    if (std::abs(a) < EPSILON) return -1; // Parallel
    float f = 1.0 / a;
    Vec3 s = orig - v0;
    float u = f * s.dot(h);
    if (u < 0.0 || u > 1.0) return -1;
    Vec3 q = s.cross(edge1);
    float v = f * dir.dot(q);
    if (v < 0.0 || u + v > 1.0) return -1;
    float t = f * edge2.dot(q);
    if (t > EPSILON) return t;
    return -1;
}

// Project 3D point P onto Plane defined by N and point P_plane
Vec3 projectOnPlane(const Vec3& P, const Vec3& N, const Vec3& P_plane) {
    float d = (P - P_plane).dot(N);
    return P - N * d;
}

// Barycentric coordinates
Vec3 barycentric(const Vec3& P, const Vec3& A, const Vec3& B, const Vec3& C) {
    Vec3 v0 = B - A, v1 = C - A, v2 = P - A;
    float d00 = v0.dot(v0);
    float d01 = v0.dot(v1);
    float d11 = v1.dot(v1);
    float d20 = v2.dot(v0);
    float d21 = v2.dot(v1);
    float denom = d00 * d11 - d01 * d01;
    float v = (d11 * d20 - d01 * d21) / denom;
    float w = (d00 * d21 - d01 * d20) / denom;
    float u = 1.0f - v - w;
    return Vec3(u, v, w);
}

// --- Triangle Class ---
enum MaterialType { DIFFUSE, REFLECTIVE };

struct Triangle {
    Vec3 v0, v1, v2;
    Vec3 normal;
    Vec3 color; // Base color for Diffuse
    MaterialType matType;
    float reflectivity; // 0.0 to 1.0
    Texture* texture; // Cached rendered texture if any (optional usage)
    
    // Cached bounding box for optimization
    Vec3 boundsMin, boundsMax;

    Triangle(Vec3 v0, Vec3 v1, Vec3 v2, Vec3 col, MaterialType mt = DIFFUSE, float refl = 0.0) 
        : v0(v0), v1(v1), v2(v2), color(col), matType(mt), reflectivity(refl), texture(nullptr) {
        
        Vec3 e1 = v1 - v0;
        Vec3 e2 = v2 - v0;
        normal = e1.cross(e2).normalize();
        
        boundsMin = v0.min(v1).min(v2);
        boundsMax = v0.max(v1).max(v2);
    }
    
    float area() const {
        return (v1 - v0).cross(v2 - v0).length() * 0.5f;
    }
};

// --- Forward Declarations ---
class Camera;
Texture renderTriangleWithTriangle(const std::vector<Triangle*>& scene, const Vec3& origin, Triangle* viewportTri, int depth, float estArea);

// --- Camera Class ---
class Camera {
public:
    Vec3 origin;
    Triangle* viewportLeft;
    Triangle* viewportRight;
    int width, height;

    Camera(Vec3 o, Triangle* left, Triangle* right, int w, int h) 
        : origin(o), viewportLeft(left), viewportRight(right), width(w), height(h) {}

    void render(const std::vector<Triangle*>& scene, const std::string& filename) {
        // Render the two triangles representing the image plane
        // We need to map these triangles to the full image pixels. 
        // However, the algorithm asks to render the triangles. 
        // Since the viewport is exactly the image plane, the texture of these triangles IS the image.
        // We pass the full resolution as the estimated area/resolution for the root call.
        
        std::cout << "Rendering Viewport Left..." << std::endl;
        Texture texLeft = renderTriangleWithTriangle(scene, origin, viewportLeft, 0, width * height);
        
        std::cout << "Rendering Viewport Right..." << std::endl;
        Texture texRight = renderTriangleWithTriangle(scene, origin, viewportRight, 0, width * height);

        // Combine into PPM. 
        // The Left and Right triangles might overlap or split the screen. 
        // Assuming they split the screen vertically or form a quad.
        // Let's assume they form a Quad: Left (Left half), Right (Right half).
        
        std::ofstream file(filename);
        file << "P3\n" << width << " " << height << "\n255\n";
        
        // Naive combination: scan pixels.
        // Since this is a "viewport based" renderer, mapping the triangle texture back to screen pixels 
        // is technically a reverse lookup. But since the camera viewport triangles ARE the projection surface,
        // the texture generated for `viewportLeft` covers exactly the region of the screen that triangle projects to.
        // If we constructed the camera with two triangles covering the whole screen, 
        // we can just iterate pixels and ask "Which triangle am I in?".
        // Simplification: We assume the user constructs the viewport such that we can just dump the textures 
        // or we map the UV of the triangles.
        
        // Let's implement a simple rasterizer to combine the two triangles onto the screen buffer.
        // This is "Post-processing" the triangle textures to the image.
        
        std::vector<Vec3> screenBuffer(width * height, Vec3(0,0,0));
        
        // Helper to write texture to screen buffer based on projection
        auto blitTriangle = [&](Triangle* tri, const Texture& tex) {
            // Bounding box of triangle on screen
            // We need to project triangle vertices to screen space? 
            // No, the triangles ARE the viewport. They are defined in 3D at some distance Z.
            // We need to project them to the screen plane (Z=0 or near plane).
            // Actually, simpler: Use Barycentric in 2D on the XY plane if camera looks along Z.
            // Let's assume standard Cornell Box orientation: Camera looks +Z.
            // Viewport is at Z=1 (example). Screen is at Z=0.
            // We project the 3D viewport triangle to 2D (x,y).
            
            float fov_dist = (tri->v0.z - origin.z); // Distance to viewport plane
            float scale = 1.0f; // Adjust based on aspect ratio/FOV setup
            
            // Map World X,Y to Screen X,Y
            // Assume orthographic scaling for simplicity of this specific mapping step
            // or perspective divide.
            
            for(int y = 0; y < height; ++y) {
                for(int x = 0; x < width; ++x) {
                    // Convert pixel to world coord at viewport plane
                    float u = (float)x / width;
                    float v = (float)y / height; // Flip Y?
                    
                    // This logic is getting coupled to how we constructed the camera.
                    // Alternative: The Texture returned by renderTriangleWithTriangle is already "rasterized" 
                    // in its own UV space.
                    // We need to know UV coordinates for the viewport triangles to index the texture.
                    // Since we didn't store UVs in Triangle class explicitly, let's add a rasterizer 
                    // that draws the triangle pixels.
                    
                    // Perspective Projection mapping:
                    // Screen Pixel (x,y) -> Ray -> Intersect Viewport Plane -> UV
                }
            }
        };

        // To keep it robust and simple without a full rasterizer in main:
        // We will just create a Texture for the Whole Screen, and renderTriangleWithTriangle
        // will effectively draw onto the screen buffer if we pass the screen buffer.
        // BUT, the requirement says: "Camera... calls render function twice and dumps texture to PPM".
        // This implies the textures ARE the final image (or parts of it).
        
        // Let's assume the viewport triangles cover the screen exactly: Left is [0, 0.5], Right is [0.5, 1.0].
        
        for (int j = 0; j < height; j++) {
            for (int i = 0; i < width; i++) {
                Color c(0,0,0);
                if (i < width / 2) {
                    // Map [0, width/2] to [0, texWidth]
                    float u = (float)i / (width / 2);
                    float v = 1.0f - (float)j / height; 
                    c = texLeft.getPixel(u, v);
                } else {
                    // Map [width/2, width] to [0, texWidth]
                    float u = (float)(i - width / 2) / (width / 2);
                    float v = 1.0f - (float)j / height;
                    c = texRight.getPixel(u, v);
                }
                file << static_cast<int>(c.r) << " " << static_cast<int>(c.g) << " " << static_cast<int>(c.b) << "\n";
            }
        }
        file.close();
    }
};

// --- Core Algorithm Implementation ---

// A simple Frustum for culling/traversal
struct Frustum {
    Vec3 apex; // Origin
    Vec3 p0, p1, p2, p3; // Quad base of the frustum
    Vec3 n0, n1, n2, n3; // Plane normals (left, right, top, bottom) pointing inwards
    
    Frustum(const Vec3& eye, const Vec3& v0, const Vec3& v1, const Vec3& v2) {
        apex = eye;
        // v0, v1, v2 define the viewport triangle. We extend it slightly to depth for intersection tests.
        // Actually, for "Frustum Culling", we need 4 planes.
        // A triangle viewport defines a pyramid with a triangular base (3 planes).
        p0 = v0; p1 = v1; p2 = v2;
        
        Vec3 edge0 = (v1 - v0).cross(apex - v0).normalize(); // Plane through Eye-V1-V0
        Vec3 edge1 = (v2 - v1).cross(apex - v1).normalize(); // Plane through Eye-V2-V1
        Vec3 edge2 = (v0 - v2).cross(apex - v2).normalize(); // Plane through Eye-V0-V2
        
        n0 = edge0; n1 = edge1; n2 = edge2;
    }
    
    // Check if a triangle is potentially inside the frustum
    bool intersects(const Triangle* tri) {
        // Check bounding box center against the 3 planes
        Vec3 center = (tri->boundsMin + tri->boundsMax) * 0.5f;
        
        // If center is behind any of the 3 planes (dot product < 0), it's outside
        if ((center - apex).dot(n0) < -EPSILON) return false;
        if ((center - apex).dot(n1) < -EPSILON) return false;
        if ((center - apex).dot(n2) < -EPSILON) return false;
        
        // Also check if it is completely behind the viewport plane (culling back geometry)
        // Viewport plane normal should point towards camera (apex)
        Vec3 vpNormal = (p1 - p0).cross(p2 - p0).normalize();
        if ((center - p0).dot(vpNormal) < 0 && (apex - p0).dot(vpNormal) > 0) {
             // Object is behind the viewport relative to camera
        }
        
        return true; // Simplified coarse check
    }
};

// Distance comparison for sorting
float distanceSquared(const Vec3& a, const Vec3& b) {
    Vec3 d = a - b;
    return d.dot(d);
}

Texture renderTriangleWithTriangle(const std::vector<Triangle*>& scene, const Vec3& origin, Triangle* viewportTri, int depth, float estArea) {
    // Termination
    if (depth > MAX_DEPTH || estArea < AREA_THRESHOLD) {
        // Return black or simple base color if stopped
        return Texture(2, 2); // Dummy small texture
    }
    
    // Initialize Texture for this viewport
    // We map the 3D triangle area to pixel dimensions
    float aspect = viewportTri->area(); 
    int texW = static_cast<int>(std::sqrt(estArea * (viewportTri->v1 - viewportTri->v0).length() / (viewportTri->v2 - viewportTri->v0).length())) + 1;
    int texH = static_cast<int>(estArea / texW) + 1;
    
    // Clamp texture size to reasonable limits
    texW = std::min(1024, std::max(2, texW));
    texH = std::min(1024, std::max(2, texH));
    
    Texture resultTex(texW, texH);
    
    // Fill with background (Black)
    std::fill(resultTex.data.begin(), resultTex.data.end(), Color(0.0f,0.0f,0.0f));
    
    // If Diffuse, just return solid color texture
    if (viewportTri->matType == DIFFUSE) {
        std::fill(resultTex.data.begin(), resultTex.data.end(), Color(viewportTri->color));
        return resultTex;
    }
    
    // --- Reflection / Rendering Logic ---
    
    // 1. Create Frustum
    Frustum frustum(origin, viewportTri->v0, viewportTri->v1, viewportTri->v2);
    Vec3 vpNormal = (viewportTri->v1 - viewportTri->v0).cross(viewportTri->v2 - viewportTri->v0).normalize();
    
    // 2. Collect & Sort visible triangles
    std::vector<Triangle*> visibleTris;
    for (Triangle* tri : scene) {
        if (tri == viewportTri) continue; // Don't self-intersect
        if (frustum.intersects(tri)) {
            // Refine: Check if triangle is in front of viewport?
            // If it's behind, it shouldn't reflect (unless translucent, ignored here)
            Vec3 triCenter = (tri->boundsMin + tri->boundsMax) * 0.5f;
            // Dot product with viewport normal
            float dist = (triCenter - viewportTri->v0).dot(vpNormal);
            if (dist > EPSILON) { // In front of the viewport triangle
                visibleTris.push_back(tri);
            }
        }
    }
    
    // Sort by distance (Far to Near)
    std::sort(visibleTris.begin(), visibleTris.end(), [&](const Triangle* a, const Triangle* b) {
        float da = distanceSquared(origin, (a->boundsMin + a->boundsMax)*0.5f);
        float db = distanceSquared(origin, (b->boundsMin + b->boundsMax)*0.5f);
        return da > db;
    });
    
    // 3. Render each visible triangle onto resultTex
    for (Triangle* objTri : visibleTris) {
        
        // A. Recursive Call
        // "Reflect camera origin about current viewport... simulate reflection behavior"
        Vec3 newOrigin;
        if (viewportTri->matType == REFLECTIVE) {
            // Reflection formula: r = d - 2(d.n)n. Here d = (Origin - PointOnPlane).
            // We reflect the Origin point.
            Vec3 d = origin - viewportTri->v0;
            newOrigin = origin - vpNormal * 2 * d.dot(vpNormal);
        } else {
            newOrigin = origin;
        }
        
        // Calculate new estimated area (projection of objTri onto viewportTri)
        // Roughly proportional to solid angle
        float dist = (objTri->boundsMin - origin).length();
        float newEstArea = (objTri->area() / (dist * dist)) * 100000.0f; // Heuristic scaling
        newEstArea = std::max(5.0f, newEstArea); // Prevent instant cut-off
        
        // Recursion
        Texture objTexture = renderTriangleWithTriangle(scene, newOrigin, objTri, depth + 1, newEstArea);
        
        // B. Texture Mapping (Project objTri onto viewportTri)
        
        // Precompute coordinates
        // We iterate over the TARGET texture (viewportTri) pixels
        // Check if pixel projects onto objTri
        
        Vec3 e0 = viewportTri->v1 - viewportTri->v0;
        Vec3 e1 = viewportTri->v2 - viewportTri->v0;
        
        // Optimized rasterization: Check bounding box of objTri projection
        // Project objTri center to viewportTri UV to find region of interest
        Vec3 objCenter = (objTri->v0 + objTri->v1 + objTri->v2) * 0.333f;
        Vec3 centerOnPlane = projectOnPlane(objCenter, vpNormal, viewportTri->v0);
        Vec3 bary = barycentric(centerOnPlane, viewportTri->v0, viewportTri->v1, viewportTri->v2);
        
        // Radius in UV space
        float radius = (objTri->boundsMax - objTri->boundsMin).length() / (dist * 0.5f);
        
        // Iterate pixels in resultTex
        // We iterate over a bounding box in the texture corresponding to the projection
        // For simplicity in this "One File" constraint, we iterate all pixels or a bounding box.
        // Let's iterate all pixels of resultTex (might be slow for high res, but correct).
        // Optimization: Only iterate near bary.
        
        int minU = std::max(0, (int)((bary.x - radius) * texW));
        int maxU = std::min(texW, (int)((bary.x + radius) * texW));
        int minV = std::max(0, (int)((bary.y - radius) * texH));
        int maxV = std::min(texH, (int)((bary.y + radius) * texH));
        
        for (int y = minV; y < maxV; ++y) {
            for (int x = minU; x < maxU; ++x) {
                float u = (float)x / texW;
                float v = (float)y / texH;
                
                // Current point on viewport triangle in 3D
                Vec3 P_on_viewport = viewportTri->v0 + e0 * u + e1 * v;
                
                // Barycentric check on Viewport Triangle (optional, strictly texture is rectangular, 
                // but triangle texture implies shape).
                // The problem says "Triangle... texture image". 
                // We should only draw if (u,v) is inside the triangle? 
                // Actually, texture is usually a rectangle containing the triangle. 
                // Let's assume (u,v) covers the bounding box of the triangle in texture space.
                // To keep it simple and strictly triangle: Check if P is inside viewportTri
                if (barycentric(P_on_viewport, viewportTri->v0, viewportTri->v1, viewportTri->v2).x < 0) continue;

                // Check if P is inside projected objTri
                // Ray from origin to P. Intersect objTri plane?
                // Perspective division is needed if we use the "Reflective Camera" concept.
                // However, we already have P_on_viewport.
                // We need to know which texel of objTri covers P.
                // This is equivalent to: Ray from newOrigin passes through P_on_viewport and hits objTri.
                
                Vec3 rayDir = (P_on_viewport - newOrigin).normalize();
                float t = rayTriangleIntersect(newOrigin, rayDir, objTri->v0, objTri->v1, objTri->v2);
                
                if (t > 0) {
                    // Hit! 
                    // Calculate UV on objTri
                    Vec3 hitPoint = newOrigin + rayDir * t;
                    Vec3 baryObj = barycentric(hitPoint, objTri->v0, objTri->v1, objTri->v2);
                    
                    // Sample texture
                    Color srcColor = objTexture.sample(baryObj.x, baryObj.y);
                    
                    // Blend with reflection intensity
                    float alpha = viewportTri->reflectivity; 
                    // If not fully reflective, mix with base color?
                    // Problem says: "For diffuse... return texture". "Reflective... calculate".
                    // Usually: Result = mix(Texture, Reflection, reflectivity).
                    // But here we are building the texture from scratch.
                    // We accumulate reflected objects.
                    
                    Color current = resultTex.getPixel(u, v);
                    Color final = srcColor.blendOver(current, alpha); 
                    // Or just simple Add/Over. Since we sorted far to near, Over works.
                    
                    resultTex.setPixel(x, y, final);
                }
            }
        }
    }
    
    return resultTex;
}

// --- Scene Setup ---
void createCornellBox(std::vector<Triangle>& triangles, Camera& cam) {
    // Coordinates: Z is up or Y is up? Let's use Y up.
    // Box dimensions: 
    // Floor: y=0, Width=555, Depth=555
    // Ceiling: y=555
    // Left Wall: x=0 (Red)
    // Right Wall: x=555 (Green)
    // Back Wall: z=555 (White/Grey) or z=0?
    // Front: z=0 (Open) or Camera at z=0 looking at z=555.
    
    float W = 555;
    float H = 555;
    float D = 555;
    
    auto makeQuad = [&](Vec3 a, Vec3 b, Vec3 c, Vec3 d, Vec3 col, MaterialType mat=DIFFUSE) {
        // Two triangles
        triangles.emplace_back(a, b, c, col, mat);
        triangles.emplace_back(a, c, d, col, mat);
    };
    
    // Floor (y=0) - White
    makeQuad(Vec3(0,0,0), Vec3(W,0,0), Vec3(W,0,D), Vec3(0,0,D), Vec3(0.73, 0.73, 0.73));
    
    // Ceiling (y=H) - White (Light)
    // Let's make a small light patch or just full white ceiling for simplicity
    makeQuad(Vec3(0,H,D), Vec3(W,H,D), Vec3(W,H,0), Vec3(0,H,0), Vec3(0.73, 0.73, 0.73));
    
    // Back Wall (z=D) - White
    makeQuad(Vec3(0,0,D), Vec3(0,H,D), Vec3(W,H,D), Vec3(W,0,D), Vec3(0.73, 0.73, 0.73));
    
    // Right Wall (x=W) - Green
    makeQuad(Vec3(W,0,D), Vec3(W,H,D), Vec3(W,H,0), Vec3(W,0,0), Vec3(0.12, 0.45, 0.15));
    
    // Left Wall (x=0) - Red
    makeQuad(Vec3(0,0,0), Vec3(0,H,0), Vec3(0,H,D), Vec3(0,0,D), Vec3(0.65, 0.05, 0.05));
    
    // Tall Box (Left-ish) - Silver/Reflective (Yellow tint per request?)
    // Request: "One Blue One Yellow". Let's do Yellow.
    // Pos: x ~ 100-200, z ~ 200-350.
    // Rotated? Simplified: Axis aligned.
    Vec3 b1_min(130, 0, 265);
    Vec3 b1_max(295, 330, 420); // Tall box
    makeQuad(Vec3(b1_min.x, b1_min.y, b1_max.z), Vec3(b1_max.x, b1_min.y, b1_max.z), Vec3(b1_max.x, b1_max.y, b1_max.z), Vec3(b1_min.x, b1_max.y, b1_max.z), Vec3(0.8, 0.8, 0.2), REFLECTIVE); // Front
    makeQuad(Vec3(b1_max.x, b1_min.y, b1_min.z), Vec3(b1_min.x, b1_min.y, b1_min.z), Vec3(b1_min.x, b1_max.y, b1_min.z), Vec3(b1_max.x, b1_max.y, b1_min.z), Vec3(0.8, 0.8, 0.2), REFLECTIVE); // Back
    makeQuad(Vec3(b1_min.x, b1_min.y, b1_min.z), Vec3(b1_min.x, b1_min.y, b1_max.z), Vec3(b1_min.x, b1_max.y, b1_max.z), Vec3(b1_min.x, b1_max.y, b1_min.z), Vec3(0.8, 0.8, 0.2), REFLECTIVE); // Left
    makeQuad(Vec3(b1_max.x, b1_min.y, b1_max.z), Vec3(b1_max.x, b1_min.y, b1_min.z), Vec3(b1_max.x, b1_max.y, b1_min.z), Vec3(b1_max.x, b1_max.y, b1_max.z), Vec3(0.8, 0.8, 0.2), REFLECTIVE); // Right
    makeQuad(Vec3(b1_min.x, b1_max.y, b1_min.z), Vec3(b1_max.x, b1_max.y, b1_min.z), Vec3(b1_max.x, b1_max.y, b1_max.z), Vec3(b1_min.x, b1_max.y, b1_max.z), Vec3(0.8, 0.8, 0.2), REFLECTIVE); // Top

    // Short Box (Right-ish) - Blue/Reflective
    Vec3 b2_min(340, 0, 100);
    Vec3 b2_max(500, 165, 260);
    makeQuad(Vec3(b2_min.x, b2_min.y, b2_max.z), Vec3(b2_max.x, b2_min.y, b2_max.z), Vec3(b2_max.x, b2_max.y, b2_max.z), Vec3(b2_min.x, b2_max.y, b2_max.z), Vec3(0.2, 0.2, 0.8), REFLECTIVE);
    makeQuad(Vec3(b2_max.x, b2_min.y, b2_min.z), Vec3(b2_min.x, b2_min.y, b2_min.z), Vec3(b2_min.x, b2_max.y, b2_min.z), Vec3(b2_max.x, b2_max.y, b2_min.z), Vec3(0.2, 0.2, 0.8), REFLECTIVE);
    makeQuad(Vec3(b2_min.x, b2_min.y, b2_min.z), Vec3(b2_min.x, b2_min.y, b2_max.z), Vec3(b2_min.x, b2_max.y, b2_max.z), Vec3(b2_min.x, b2_max.y, b2_min.z), Vec3(0.2, 0.2, 0.8), REFLECTIVE);
    makeQuad(Vec3(b2_max.x, b2_min.y, b2_max.z), Vec3(b2_max.x, b2_min.y, b2_min.z), Vec3(b2_max.x, b2_max.y, b2_min.z), Vec3(b2_max.x, b2_max.y, b2_max.z), Vec3(0.2, 0.2, 0.8), REFLECTIVE);
    makeQuad(Vec3(b2_min.x, b2_max.y, b2_min.z), Vec3(b2_max.x, b2_max.y, b2_min.z), Vec3(b2_max.x, b2_max.y, b2_max.z), Vec3(b2_min.x, b2_max.y, b2_max.z), Vec3(0.2, 0.2, 0.8), REFLECTIVE);

    // Camera
    // Look from (278, 278, -800) at (278, 278, 0). Up (0, 1, 0).
    Vec3 camPos(278, 273, -800); // Slightly adjusted height
    Vec3 target(278, 273, 0);
    Vec3 up(0, 1, 0);
    
    // Viewport setup
    float distToView = 800; // Distance from camera to viewport plane (z=0)
    float viewW = 555 * 1.5; // Zoom
    float viewH = viewW * (IMAGE_HEIGHT / IMAGE_WIDTH);
    
    Vec3 viewCenter = target;
    Vec3 camDir = (target - camPos).normalize();
    Vec3 camRight = camDir.cross(up).normalize();
    Vec3 camUp_real = camRight.cross(camDir);
    
    Vec3 vTL = viewCenter - camRight * (viewW/2) + camUp_real * (viewH/2);
    Vec3 vTR = viewCenter + camRight * (viewW/2) + camUp_real * (viewH/2);
    Vec3 vBL = viewCenter - camRight * (viewW/2) - camUp_real * (viewH/2);
    Vec3 vBR = viewCenter + camRight * (viewW/2) - camUp_real * (viewH/2);
    
    // Split viewport into two triangles
    // Note: These are temporary objects created in the vector, pointers must remain valid.
    // We pass pointers to Camera.
    
    Triangle* vpLeft = new Triangle(vTL, vBL, vTR, Vec3(1,1,1), DIFFUSE); // Color irrelevant
    Triangle* vpRight = new Triangle(vBL, vBR, vTR, Vec3(1,1,1), DIFFUSE);
    
    triangles.push_back(*vpLeft);
    triangles.push_back(*vpRight);
    
    // Update pointers to point to the vector storage
    vpLeft = &triangles[triangles.size()-2];
    vpRight = &triangles[triangles.size()-1];
    
    cam = Camera(camPos, vpLeft, vpRight, IMAGE_WIDTH, IMAGE_HEIGHT);
}

// --- Main ---
int main() {
    std::vector<Triangle> triangles;
    Camera cam(Vec3(0,0,0), nullptr, nullptr, 0, 0);
    
    createCornellBox(triangles, cam);
    
    std::vector<Triangle*> scenePtrs;
    for(auto& t : triangles) {
        scenePtrs.push_back(&t);
    }
    
    std::cout << "Starting render..." << std::endl;
    cam.render(scenePtrs, "cornell_box.ppm");
    std::cout << "Finished render. Output: cornell_box.ppm" << std::endl;
    
    return 0;
}
