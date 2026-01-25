// type_erasure_test.cpp
#include <iostream>
#include <vector>
#include <functional>
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

int main() {
    const size_t count = 400000;
    std::vector<std::function<void()>> renderers(count);

    for (size_t i = 0; i < count; ++i) {
        switch (i % 4) {
            case 0: {
                Metal m;
                renderers[i] = [m]() { m.render(); };
                break;
            }
            case 1: {
                Lambertian l;
                renderers[i] = [l]() { l.render(); };
                break;
            }
            case 2: {
                Plastic p;
                renderers[i] = [p]() { p.render(); };
                break;
            }
            case 3: {
                Glass g;
                renderers[i] = [g]() { g.render(); };
                break;
            }
        }
    }

    auto start_time = std::chrono::high_resolution_clock::now();
    for (const auto& fn : renderers) {
        fn();
    }
    auto end_time = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    std::cout << "Type Erasure (std::function) Execution Time: " << duration.count() << " µs\n";

    return 0;
}
