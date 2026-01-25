# -*- coding: utf-8 -*-
"""
简易三角形康奈尔盒渲染器（递归反射版本）

特性：
- 只用三角形面片构建场景（墙面、灯、两个盒子）
- Camera 视口被视为一个平面（理论上可拆成两个三角形）
- 漫反射表面：不递归，直接光照
- 镜面（金属）表面：递归反射，模拟金属反射
- 输出 PPM 图片：cornell.ppm
"""

import math

# ======================
# 基础向量运算
# ======================

def v_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])

def v_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])

def v_mul(a, s):
    # 向量 * 标量
    return (a[0] * s, a[1] * s, a[2] * s)

def v_mul_comp(a, b):
    # 分量乘（用于颜色调制）
    return (a[0] * b[0], a[1] * b[1], a[2] * b[2])

def v_dot(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

def v_cross(a, b):
    return (
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0]
    )

def v_length(a):
    return math.sqrt(v_dot(a, a))

def v_norm(a):
    l = v_length(a)
    if l == 0:
        return (0.0, 0.0, 0.0)
    return (a[0]/l, a[1]/l, a[2]/l)

def v_reflect(d, n):
    # 反射方向：r = d - 2*(d·n)*n
    dn = v_dot(d, n)
    return v_sub(d, v_mul(n, 2.0 * dn))

# ======================
# 材质和三角形定义
# ======================

class Material:
    def __init__(self, color=(1.0, 1.0, 1.0),
                 emission=(0.0, 0.0, 0.0),
                 kind="diffuse",
                 reflectivity=1.0):
        """
        kind: "diffuse" 或 "mirror"
        color: 基础颜色/反射率
        emission: 自发光颜色（光源）
        reflectivity: 镜面反射强度（0~1）
        """
        self.color = color
        self.emission = emission
        self.kind = kind
        self.reflectivity = reflectivity

class Triangle:
    def __init__(self, v0, v1, v2, material):
        self.v0 = v0
        self.v1 = v1
        self.v2 = v2
        self.material = material
        # 预计算几何法线
        self.normal = v_norm(v_cross(v_sub(v1, v0), v_sub(v2, v0)))

    def intersect(self, ray_o, ray_d, eps=1e-6):
        """
        Möller–Trumbore 三角形求交
        返回 (t, u, v) 或 None
        """
        v0, v1, v2 = self.v0, self.v1, self.v2
        edge1 = v_sub(v1, v0)
        edge2 = v_sub(v2, v0)
        h = v_cross(ray_d, edge2)
        a = v_dot(edge1, h)
        if -eps < a < eps:
            return None  # 平行
        f = 1.0 / a
        s = v_sub(ray_o, v0)
        u = f * v_dot(s, h)
        if u < 0.0 or u > 1.0:
            return None
        q = v_cross(s, edge1)
        v = f * v_dot(ray_d, q)
        if v < 0.0 or u + v > 1.0:
            return None
        t = f * v_dot(edge2, q)
        if t > eps:
            return (t, u, v)
        return None

# ======================
# 场景构建（康奈尔盒）
# ======================

def make_quad(v00, v10, v11, v01, material):
    """
    用两个三角形表示一个四边形：
    v00----v10
    |     /
    |    /
    v01----v11
    """
    return [
        Triangle(v00, v10, v11, material),
        Triangle(v00, v11, v01, material)
    ]

def add_box(x0, x1, y0, y1, z0, z1, material):
    """
    用 12 个三角形（6 个面 * 2）构造轴对齐长方体
    """
    tris = []
    # 8 个顶点
    p000 = (x0, y0, z0)
    p001 = (x0, y0, z1)
    p010 = (x0, y1, z0)
    p011 = (x0, y1, z1)
    p100 = (x1, y0, z0)
    p101 = (x1, y0, z1)
    p110 = (x1, y1, z0)
    p111 = (x1, y1, z1)

    # +X 面
    tris += make_quad(p100, p110, p111, p101, material)
    # -X 面
    tris += make_quad(p000, p001, p011, p010, material)
    # +Y 面（顶面）
    tris += make_quad(p010, p011, p111, p110, material)
    # -Y 面（底面）
    tris += make_quad(p000, p100, p101, p001, material)
    # +Z 面
    tris += make_quad(p001, p101, p111, p011, material)
    # -Z 面
    tris += make_quad(p000, p010, p110, p100, material)

    return tris

def build_cornell_scene():
    """
    构建一个 0~1 立方体空间内的简易康奈尔盒
    - 左墙红色，右墙绿色，其余白色
    - 顶部一个小矩形面光源
    - 场景中两个盒子：一个漫反射，一个金属镜面
    """
    scene = []

    # 材质
    white = Material(color=(0.8, 0.8, 0.8), kind="diffuse")
    red   = Material(color=(0.75, 0.15, 0.15), kind="diffuse")
    green = Material(color=(0.15, 0.75, 0.15), kind="diffuse")
    light_mat = Material(color=(1.0, 1.0, 1.0),
                         emission=(12.0, 12.0, 12.0),  # 强光
                         kind="diffuse")

    metal = Material(color=(0.9, 0.9, 0.9),
                     kind="mirror",
                     reflectivity=0.95)

    grey_box = Material(color=(0.75, 0.75, 0.75),
                        kind="diffuse")

    # 盒子边界
    minc, maxc = 0.0, 1.0

    # 地板 y=0
    scene += make_quad(
        (minc, 0.0, minc),
        (maxc, 0.0, minc),
        (maxc, 0.0, maxc),
        (minc, 0.0, maxc),
        white
    )

    # 天花板 y=1
    scene += make_quad(
        (minc, 1.0, minc),
        (minc, 1.0, maxc),
        (maxc, 1.0, maxc),
        (maxc, 1.0, minc),
        white
    )

    # 后墙 z=1
    scene += make_quad(
        (minc, minc, 1.0),
        (maxc, minc, 1.0),
        (maxc, maxc, 1.0),
        (minc, maxc, 1.0),
        white
    )

    # 左墙 x=0 （红）
    scene += make_quad(
        (0.0, minc, minc),
        (0.0, minc, maxc),
        (0.0, maxc, maxc),
        (0.0, maxc, minc),
        red
    )

    # 右墙 x=1 （绿）
    scene += make_quad(
        (1.0, minc, minc),
        (1.0, maxc, minc),
        (1.0, maxc, maxc),
        (1.0, minc, maxc),
        green
    )

    # 顶部面光源（天花板中间一小块）
    lx0, lx1 = 0.35, 0.65
    lz0, lz1 = 0.35, 0.65
    ly = 0.999  # 略低于天花板避免数值问题
    light_tris = make_quad(
        (lx0, ly, lz0),
        (lx1, ly, lz0),
        (lx1, ly, lz1),
        (lx0, ly, lz1),
        light_mat
    )
    scene += light_tris

    # 低盒子（金属反射）
    scene += add_box(
        0.20, 0.50,   # x
        0.0,  0.30,   # y
        0.20, 0.50,   # z
        metal
    )

    # 高盒子（漫反射）
    scene += add_box(
        0.55, 0.85,
        0.0,  0.70,
        0.55, 0.85,
        grey_box
    )

    return scene

# ======================
# 光线追踪核心
# ======================

def find_closest_hit(scene, ray_o, ray_d):
    """
    在所有三角形中找到最近交点
    返回 (tri, t, hit_pos, hit_normal) 或 None
    """
    closest_t = float('inf')
    hit_tri = None
    hit_pos = None
    hit_normal = None

    for tri in scene:
        res = tri.intersect(ray_o, ray_d)
        if res is None:
            continue
        t, u, v = res
        if t < closest_t:
            closest_t = t
            # 交点
            hit_pos = v_add(ray_o, v_mul(ray_d, t))
            # 几何法线拷贝一份
            n = tri.normal
            # 确保法线朝向入射光线的反方向
            if v_dot(n, ray_d) > 0:
                n = v_mul(n, -1.0)
            hit_normal = n
            hit_tri = tri

    if hit_tri is None:
        return None
    return hit_tri, closest_t, hit_pos, hit_normal

def build_lights(scene):
    """
    从场景中提取所有发光三角形
    """
    lights = []
    for tri in scene:
        if (tri.material.emission[0] > 0 or
            tri.material.emission[1] > 0 or
            tri.material.emission[2] > 0):
            lights.append(tri)
    return lights

def triangle_center(tri):
    return (
        (tri.v0[0] + tri.v1[0] + tri.v2[0]) / 3.0,
        (tri.v0[1] + tri.v1[1] + tri.v2[1]) / 3.0,
        (tri.v0[2] + tri.v1[2] + tri.v2[2]) / 3.0,
    )

def trace_ray(scene, lights, ray_o, ray_d, depth, max_depth=4, eps=1e-4):
    """
    核心递归光线追踪函数
    - 只在 kind='mirror' 时递归
    - kind='diffuse' 时只做直接光照，不再递归
    """
    if depth > max_depth:
        return (0.0, 0.0, 0.0)

    hit = find_closest_hit(scene, ray_o, ray_d)
    if hit is None:
        # 背景颜色（黑）
        return (0.0, 0.0, 0.0)

    tri, t, hit_pos, n = hit
    mat = tri.material

    # 自发光（光源），直接返回其发光颜色（主要用于第一次命中）
    color = mat.emission

    # 漫反射：只做一次直接光照
    if mat.kind == "diffuse":
        direct = (0.0, 0.0, 0.0)
        for light in lights:
            # 简化：只用光源三角形中心一个点做光照 & 阴影测试
            lp = triangle_center(light)
            to_light = v_sub(lp, hit_pos)
            dist2 = v_dot(to_light, to_light)
            dist = math.sqrt(dist2)
            l_dir = v_norm(to_light)

            # 阴影检测：从 hit_pos 向 lp 打一条 shadow ray
            shadow_o = v_add(hit_pos, v_mul(n, eps))
            shadow_hit = find_closest_hit(scene, shadow_o, l_dir)
            blocked = False
            if shadow_hit is not None:
                _, t_shadow, _, _ = shadow_hit
                if t_shadow < dist - eps:
                    blocked = True

            if not blocked:
                # 漫反射 Lambert 项
                lambert = max(0.0, v_dot(n, l_dir))
                if lambert > 0:
                    # 简单地把光源发光乘以漫反射颜色和 lambert
                    # 不严格做 1/r^2 衰减，只做一个系数缩放
                    # 可以适当乘个缩放系数让亮度合适
                    intensity = 1.0
                    # contribution = mat.color * light.emission * lambert * intensity
                    c = v_mul_comp(mat.color, light.material.emission)
                    c = v_mul(c, lambert * intensity)
                    direct = v_add(direct, c)

        color = v_add(color, direct)
        return color

    # 镜面（金属）表面：计算反射并递归
    if mat.kind == "mirror":
        refl_dir = v_reflect(ray_d, n)
        refl_o = v_add(hit_pos, v_mul(n, eps))
        refl_col = trace_ray(scene, lights, refl_o, refl_dir, depth + 1, max_depth, eps)
        # 反射颜色 = 本身颜色 * 反射颜色 * 反射率
        refl_col = v_mul_comp(mat.color, refl_col)
        refl_col = v_mul(refl_col, mat.reflectivity)
        color = v_add(color, refl_col)
        return color

    # 其他类型（未用到），直接返回自发光
    return color

# ======================
# 渲染主函数
# ======================

def clamp(x, lo=0.0, hi=1.0):
    return max(lo, min(hi, x))

def gamma_correct(c, gamma=2.2):
    inv = 1.0 / gamma
    return (c[0] ** inv, c[1] ** inv, c[2] ** inv)

def render():
    # 图像分辨率
    width = 400
    height = 400

    # 构建场景
    scene = build_cornell_scene()
    lights = build_lights(scene)

    # 相机设置
    cam_pos = (0.5, 0.5, -1.5)  # 在盒子前方
    look_at = (0.5, 0.5, 0.0)   # 看向盒子中心
    up = (0.0, 1.0, 0.0)
    fov = 40.0  # 视角（度）
    aspect = width / float(height)
    scale = math.tan(math.radians(fov * 0.5))

    # 构建相机坐标系
    forward = v_norm(v_sub(look_at, cam_pos))
    right = v_norm(v_cross(forward, up))
    true_up = v_cross(right, forward)

    framebuffer = [[(0.0, 0.0, 0.0) for _ in range(width)] for _ in range(height)]

    for y in range(height):
        print(f"Rendering line {y+1}/{height} ...", end="\r")
        for x in range(width):
            # 像素中心 NDC 坐标 [-1,1]
            px = (2.0 * ((x + 0.5) / width) - 1.0) * aspect * scale
            py = (1.0 - 2.0 * ((y + 0.5) / height)) * scale

            # 在相机坐标系中构造方向
            dir_cam = v_add(
                v_add(v_mul(right, px), v_mul(true_up, py)),
                forward
            )
            ray_d = v_norm(dir_cam)
            ray_o = cam_pos

            col = trace_ray(scene, lights, ray_o, ray_d, depth=0, max_depth=4)

            # gamma 校正
            col = gamma_correct((
                clamp(col[0]),
                clamp(col[1]),
                clamp(col[2])
            ))

            framebuffer[y][x] = col

    print("\n渲染完成，正在写入文件 cornell.ppm ...")

    # 输出 PPM（文本格式）
    with open("cornell.ppm", "w") as f:
        f.write(f"P3\n{width} {height}\n255\n")
        for y in range(height):
            for x in range(width):
                r, g, b = framebuffer[y][x]
                ir = int(255.99 * clamp(r))
                ig = int(255.99 * clamp(g))
                ib = int(255.99 * clamp(b))
                f.write(f"{ir} {ig} {ib}\n")

    print("写入完成：cornell.ppm")

if __name__ == "__main__":
    render()
