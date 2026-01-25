// baseline_asm.cpp
#include <iostream>
#include <chrono>
#include <cstddef>

int main() {
    const size_t count = 400000;

    auto start_time = std::chrono::high_resolution_clock::now();

    for (size_t i = 0; i < count; ++i) {
        asm volatile("" ::: "memory");
    }

    auto end_time = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    std::cout << "Baseline (400000x asm volatile(\"\" ::: \"memory\")) Time: "
              << duration.count() << " us\n";

    return 0;
}
