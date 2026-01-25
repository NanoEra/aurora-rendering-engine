// crtp_test.cpp
#include <iostream>
#include <vector>
#include <chrono>

template<typename Derived>
struct Material {
    void render() const {
        static_cast<const Derived*>(this)->renderImpl();
    }
};

void process(const auto& mat) {
    mat.render();
}

struct Metal : Material<Metal> {
    float data[4];
    void renderImpl() const {
        asm volatile("" ::: "memory");
    }
};

struct Lambertian : Material<Lambertian> {
    float data[4];
    void renderImpl() const {
        asm volatile("" ::: "memory");
    }
};

struct Plastic : Material<Plastic> {
    float data[4];
    void renderImpl() const {
        asm volatile("" ::: "memory");
    }
};

struct Glass : Material<Glass> {
    float data[4];
    void renderImpl() const {
        asm volatile("" ::: "memory");
    }
};

int main() {
    const size_t count = 400000;
    std::vector<Metal> metals(count / 4);
    std::vector<Lambertian> lambertians(count / 4);
    std::vector<Plastic> plastics(count / 4);
    std::vector<Glass> glasses(count / 4);
	std::vector<Material<Metal>*> material_ptrs;

    auto start_time = std::chrono::high_resolution_clock::now();

	for(const auto& m : metals) m.render();
	for(const auto& l : lambertians) l.render();
	for(const auto& p : plastics) p.render();
	for(const auto& g : glasses) g.render();

    // for (const auto& m : metals) process(m);
    // for (const auto& l : lambertians) process(l);
    // for (const auto& p : plastics) process(p);
    // for (const auto& g : glasses) process(g);

    auto end_time = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    std::cout << "CRTP Static Polymorphism Execution Time: " << duration.count() << " µs\n";

    return 0;
}
