// virtual_test.cpp
#include <iostream>
#include <vector>
#include <chrono>
#include <memory>

class Material {
public:
    virtual ~Material() = default;
    virtual void render() const = 0;
};

struct Metal : public Material {
    float data[4];
    void render() const override {
        asm volatile("" ::: "memory");
    }
};

struct Lambertian : public Material {
    float data[4];
    void render() const override {
        asm volatile("" ::: "memory");
    }
};

struct Plastic : public Material {
    float data[4];
    void render() const override {
        asm volatile("" ::: "memory");
    }
};

struct Glass : public Material {
    float data[4];
    void render() const override {
        asm volatile("" ::: "memory");
    }
};

int main() {
    const size_t count = 400000;
    std::vector<std::unique_ptr<Material>> materials(count);

    for (size_t i = 0; i < count; ++i) {
        switch(i % 4) {
            case 0: materials[i] = std::make_unique<Metal>(); break;
            case 1: materials[i] = std::make_unique<Lambertian>(); break;
            case 2: materials[i] = std::make_unique<Plastic>(); break;
            case 3: materials[i] = std::make_unique<Glass>(); break;
        }
    }

    auto start_time = std::chrono::high_resolution_clock::now();
    for (const auto& mat : materials) {
        mat->render();
    }
    auto end_time = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    std::cout << "Virtual Function Execution Time: " << duration.count() << " µs\n";

    return 0;
}
