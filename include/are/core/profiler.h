/**
 * @file profiler.h
 * @brief Performance profiling utilities
 */

#ifndef ARE_INCLUDE_CORE_PROFILER_H
#define ARE_INCLUDE_CORE_PROFILER_H

#include <are/core/types.h>
#include <string>
#include <chrono>
#include <unordered_map>

namespace are {

/**
 * @struct ProfileResult
 * @brief Result of a profiling measurement
 */
struct ProfileResult {
    std::string name_;                    ///< Profile section name
    double duration_ms_;                  ///< Duration in milliseconds
    uint64_t call_count_;                 ///< Number of times called
    double avg_duration_ms_;              ///< Average duration per call
};

/**
 * @class Profiler
 * @brief Simple performance profiler
 * 
 * This class provides basic timing functionality for performance analysis.
 * It is only active when ARE_ENABLE_PROFILING is defined.
 */
class Profiler {
public:
    /**
     * @brief Initialize the profiler
     */
    static void init();

    /**
     * @brief Shutdown the profiler and print results
     */
    static void shutdown();

    /**
     * @brief Begin a profiling section
     * @param name Section name
     */
    static void begin(const std::string& name);

	/**
     * @brief End a profiling section
     * @param name Section name
     */
    static void end(const std::string& name);

    /**
     * @brief Get profiling results
     * @return Map of section names to profile results
     */
    static const std::unordered_map<std::string, ProfileResult>& get_results();

    /**
     * @brief Reset all profiling data
     */
    static void reset();

    /**
     * @brief Print profiling results to console
     */
    static void print_results();

private:
    struct SectionData {
        std::chrono::high_resolution_clock::time_point start_time_;
        double total_duration_ms_ = 0.0;
        uint64_t call_count_ = 0;
    };

    static std::unordered_map<std::string, SectionData> sections_;
    static std::unordered_map<std::string, ProfileResult> results_;
    static bool enabled_;
};

/**
 * @class ScopedProfiler
 * @brief RAII-style profiler for automatic timing
 */
class ScopedProfiler {
public:
    /**
     * @brief Constructor - begins profiling
     * @param name Section name
     */
    explicit ScopedProfiler(const std::string& name);

    /**
     * @brief Destructor - ends profiling
     */
    ~ScopedProfiler();

private:
    std::string name_;
};

} // namespace are

// Profiling macros
#ifdef ARE_ENABLE_PROFILING
	#define ARE_PROFILE_BEGIN(name) are::Profiler::begin(name)
    #define ARE_PROFILE_END(name)   are::Profiler::end(name)
    #define ARE_PROFILE_SCOPE(name) are::ScopedProfiler are_profiler_##__LINE__(name)
    #define ARE_PROFILE_FUNCTION()  ARE_PROFILE_SCOPE(__func__)
#else
    #define ARE_PROFILE_BEGIN(name) ((void)0)
    #define ARE_PROFILE_END(name)   ((void)0)
    #define ARE_PROFILE_SCOPE(name) ((void)0)
    #define ARE_PROFILE_FUNCTION()  ((void)0)
#endif

#endif // ARE_INCLUDE_CORE_PROFILER_H
