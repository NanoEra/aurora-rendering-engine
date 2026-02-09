#!/bin/bash

echo "=========================================="
echo "Aurora Rendering Engine - System Check"
echo "=========================================="
echo ""

# Check display server
echo "1. Display Server:"
if [ -n "$DISPLAY" ]; then
    echo "   ✓ X11 detected: $DISPLAY"
elif [ -n "$WAYLAND_DISPLAY" ]; then
    echo "   ✓ Wayland detected: $WAYLAND_DISPLAY"
else
    echo "   ✗ No display server detected!"
    echo "   Please run this in a graphical environment"
fi
echo ""

# Check OpenGL
echo "2. OpenGL Information:"
if command -v glxinfo &> /dev/null; then
    GL_VERSION=$(glxinfo | grep "OpenGL version" | cut -d: -f2)
    GL_RENDERER=$(glxinfo | grep "OpenGL renderer" | cut -d: -f2)
    GL_VENDOR=$(glxinfo | grep "OpenGL vendor" | cut -d: -f2)
    
    echo "   Vendor:  $GL_VENDOR"
    echo "   Renderer:$GL_RENDERER"
    echo "   Version: $GL_VERSION"
    
    # Check version
    MAJOR=$(echo $GL_VERSION | grep -oP '\d+' | head -1)
    MINOR=$(echo $GL_VERSION | grep -oP '\d+' | head -2 | tail -1)
    
    if [ "$MAJOR" -ge 4 ] && [ "$MINOR" -ge 5 ]; then
        echo "   ✓ OpenGL 4.5+ supported"
    else
        echo "   ✗ OpenGL 4.5+ NOT supported (found $MAJOR.$MINOR)"
        echo "   This engine requires OpenGL 4.5 or higher"
    fi
else
    echo "   ✗ glxinfo not found (install mesa-utils)"
fi
echo ""

# Check GLFW
echo "3. GLFW Library:"
if ldconfig -p | grep -q libglfw; then
    echo "   ✓ GLFW library found"
else
    echo "   ✗ GLFW library not found"
    echo "   Install: sudo apt-get install libglfw3-dev"
fi
echo ""

# Check GLM
echo "4. GLM Library:"
if [ -d "/usr/include/glm" ] || [ -d "/usr/local/include/glm" ]; then
    echo "   ✓ GLM headers found"
else
    echo "   ✗ GLM headers not found"
    echo "   Install: sudo apt-get install libglm-dev"
fi
echo ""

# Check GPU
echo "5. GPU Information:"
if command -v lspci &> /dev/null; then
    GPU=$(lspci | grep -i vga | cut -d: -f3)
    echo "   GPU:$GPU"
else
    echo "   ✗ lspci not found"
fi
echo ""

# Check drivers
echo "6. Graphics Drivers:"
if lsmod | grep -q nvidia; then
    echo "   ✓ NVIDIA driver loaded"
    if command -v nvidia-smi &> /dev/null; then
        DRIVER_VERSION=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader)
        echo "   Driver version: $DRIVER_VERSION"
    fi
elif lsmod | grep -q amdgpu; then
    echo "   ✓ AMD driver (amdgpu) loaded"
elif lsmod | grep -q i915; then
    echo "   ✓ Intel driver (i915) loaded"
else
    echo "   ? Unknown driver"
fi
echo ""

echo "=========================================="
echo "System check complete"
echo "=========================================="
