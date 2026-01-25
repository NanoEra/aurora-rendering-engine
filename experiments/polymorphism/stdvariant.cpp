// variant_test.cpp
#include <iostream>
#include <vector>
#include <variant>
#include <chrono>

struct Metal {
    float data[4];
    void render() const {
        asm volatile("" ::: "memory");
    }
};

struct Lambertian {
    float data[4];
    void render() const {
        asm volatile("" ::: "memory");
    }
};

struct Plastic {
    float data[4];
    void render() const {
        asm volatile("" ::: "memory");
    }
};

struct Glass {
    float data[4];
    void render() const {
        asm volatile("" ::: "memory");
    }
};

using Material = std::variant<Metal, Lambertian, Plastic, Glass>;

void visitMaterial(Material& mat) {
    std::visit([](auto& m) {
        m.render();
    }, mat);
}

int main() {
    const size_t count = 400000;
    std::vector<Material> materials(count);

    for (size_t i = 0; i < count; ++i) {
        switch (i % 4) {
            case 0: materials[i] = Metal{{static_cast<float>(i), 0, 0, 0}}; break;
            case 1: materials[i] = Lambertian{{static_cast<float>(i), 0, 0, 0}}; break;
            case 2: materials[i] = Plastic{{static_cast<float>(i), 0, 0, 0}}; break;
            case 3: materials[i] = Glass{{static_cast<float>(i), 0, 0, 0}}; break;
        }
    }

    auto start_time = std::chrono::high_resolution_clock::now();
    for (auto& mat : materials) {
        visitMaterial(mat);
    }
    auto end_time = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    std::cout << "std::variant Execution Time: " << duration.count() << " µs\n";

    return 0;
}
