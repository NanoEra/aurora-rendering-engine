// ecs_test.cpp
#include <iostream>
#include <vector>
#include <chrono>
#include <algorithm>

enum class MaterialType {
    Metal,
    Lambertian,
    Plastic,
    Glass
};

struct MaterialComponent {
    MaterialType type;
    float data[4]; // 模拟复杂状态
};

void renderMetal(const MaterialComponent&) {
    asm volatile("" ::: "memory"); // 防止编译器过度优化，模拟运算
}

void renderLambertian(const MaterialComponent&) {
    asm volatile("" ::: "memory");
}

void renderPlastic(const MaterialComponent&) {
    asm volatile("" ::: "memory");
}

void renderGlass(const MaterialComponent&) {
    asm volatile("" ::: "memory");
}

void processECS(std::vector<MaterialComponent>& materials) {
    // 按类型排序
    // std::sort(materials.begin(), materials.end(),
    //           [](const MaterialComponent& a, const MaterialComponent& b) {
    //               return a.type < b.type;
    //           });

    size_t start = 0;
    for (size_t i = 1; i <= materials.size(); ++i) {
        if (i == materials.size() || materials[i].type != materials[start].type) {
            for (size_t j = start; j < i; ++j) {
                switch (materials[j].type) {
                    case MaterialType::Metal: renderMetal(materials[j]); break;
                    case MaterialType::Lambertian: renderLambertian(materials[j]); break;
                    case MaterialType::Plastic: renderPlastic(materials[j]); break;
                    case MaterialType::Glass: renderGlass(materials[j]); break;
                }
            }
            start = i;
        }
    }
}

int main() {
    const size_t count = 400000;
    std::vector<MaterialComponent> materials(count);

    for (size_t i = 0; i < count; ++i) {
        materials[i].type = static_cast<MaterialType>(i % 4);
        for (int j = 0; j < 4; ++j)
            materials[i].data[j] = static_cast<float>(i + j);
    }

    auto start_time = std::chrono::high_resolution_clock::now();
    processECS(materials);
    auto end_time = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    std::cout << "ECS Execution Time: " << duration.count() << " µs\n";

    return 0;
}
