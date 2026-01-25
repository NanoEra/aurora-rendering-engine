#include <chrono>
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

// -----------------------------
// 1) 类型擦除：一个稳定的“句柄”类型
//    - 内部：void* + render_fn + destroy_fn
//    - move-only，语义类似 unique_ptr
// -----------------------------
struct MaterialHandle {
    using RenderFn = void (*)(const void*) noexcept;
    using DestroyFn = void (*)(void*) noexcept;

    void* ptr = nullptr;
    RenderFn render_fn = nullptr;
    DestroyFn destroy_fn = nullptr;

    MaterialHandle() = default;

    MaterialHandle(void* p, RenderFn r, DestroyFn d) noexcept
        : ptr(p), render_fn(r), destroy_fn(d) {}

    MaterialHandle(const MaterialHandle&) = delete;
    MaterialHandle& operator=(const MaterialHandle&) = delete;

    MaterialHandle(MaterialHandle&& other) noexcept
        : ptr(other.ptr), render_fn(other.render_fn), destroy_fn(other.destroy_fn) {
        other.ptr = nullptr;
        other.render_fn = nullptr;
        other.destroy_fn = nullptr;
    }

    MaterialHandle& operator=(MaterialHandle&& other) noexcept {
        if (this == &other) return *this;
        reset();
        ptr = other.ptr;
        render_fn = other.render_fn;
        destroy_fn = other.destroy_fn;
        other.ptr = nullptr;
        other.render_fn = nullptr;
        other.destroy_fn = nullptr;
        return *this;
    }

    ~MaterialHandle() { reset(); }

    void reset() noexcept {
        if (ptr) {
            destroy_fn(ptr);
        }
        ptr = nullptr;
        render_fn = nullptr;
        destroy_fn = nullptr;
    }

    void render() const noexcept {
        // 假设创建时已填好函数指针
        render_fn(ptr);
    }

    template <class T, class... Args>
    static MaterialHandle make(Args&&... args) {
        T* obj = new T(std::forward<Args>(args)...);
        return MaterialHandle{
            obj,
            [](const void* p) noexcept {
                static_cast<const T*>(p)->render();
            },
            [](void* p) noexcept {
                delete static_cast<T*>(p);
            },
        };
    }
};

// 你要求用 std::variant：但不枚举 Metal/Lambertian/...
// variant 只放一个稳定包装类型即可，永远不需要随扩展修改。
using Material = std::variant<std::monostate, MaterialHandle>;

// -----------------------------
// 2) 注册工厂：通过 id -> creator() 创建 Material
// -----------------------------
class MaterialFactory {
public:
    using Creator = Material (*)();

    static MaterialFactory& instance() {
        static MaterialFactory f;
        return f;
    }

    void reg(int id, Creator creator) {
        creators_[id] = creator;
    }

    Material create(int id) const {
        auto it = creators_.find(id);
        if (it == creators_.end()) {
            throw std::runtime_error("MaterialFactory: unknown material id");
        }
        return (it->second)();
    }

private:
    std::unordered_map<int, Creator> creators_;
};

template <class T>
struct MaterialRegistrar {
    explicit MaterialRegistrar(int id) {
        // 无捕获 lambda 可转换为函数指针 Creator
        MaterialFactory::instance().reg(id, []() -> Material {
            return Material{MaterialHandle::make<T>()};
        });
    }
};

// -----------------------------
// 3) “派生类”：不继承基类、不写虚函数
//    只需要提供 render() 即可
// -----------------------------
struct Metal {
    float data[4]{};
    void render() const noexcept { asm volatile("" ::: "memory"); }
};

struct Lambertian {
    float data[4]{};
    void render() const noexcept { asm volatile("" ::: "memory"); }
};

struct Plastic {
    float data[4]{};
    void render() const noexcept { asm volatile("" ::: "memory"); }
};

struct Glass {
    float data[4]{};
    void render() const noexcept { asm volatile("" ::: "memory"); }
};

// -----------------------------
// 4) 注册：扩展新类型时，最多“注册一下”
//    你也可以把注册放到各自 .cpp 里，做到模块化插拔。
// -----------------------------
static MaterialRegistrar<Metal> reg_metal{0};
static MaterialRegistrar<Lambertian> reg_lambertian{1};
static MaterialRegistrar<Plastic> reg_plastic{2};
static MaterialRegistrar<Glass> reg_glass{3};

// -----------------------------
// 5) 实验：创建 + 调用 render() 测时
// -----------------------------
int main() {
    const std::size_t count = 400000;

    std::vector<Material> materials;
    materials.reserve(count);

    for (std::size_t i = 0; i < count; ++i) {
        const int id = static_cast<int>(i % 4);
        materials.emplace_back(MaterialFactory::instance().create(id));
    }

    const auto start_time = std::chrono::high_resolution_clock::now();

    for (const auto& mat : materials) {
        std::visit(
            [](const auto& x) noexcept {
                using X = std::decay_t<decltype(x)>;
                if constexpr (std::is_same_v<X, std::monostate>) {
                    // 空对象：不做任何事
                } else {
                    x.render(); // 函数指针调用，不走虚表
                }
            },
            mat);
    }

    const auto end_time = std::chrono::high_resolution_clock::now();
    const auto duration =
        std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    std::cout << "Variant+Registry Execution Time: " << duration.count() << " us\n";
    return 0;
}
