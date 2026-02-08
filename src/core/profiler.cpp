/**
 * @file profiler.cpp
 * @brief Implementation of performance profiler
 */

#include <are/core/profiler.h>
#include <are/core/logger.h>
#include <iomanip>
#include <sstream>
#include <algorithm>

namespace are {

std::unordered_map<std::string, Profiler::SectionData> Profiler::sections_;
std::unordered_map<std::string, ProfileResult> Profiler::results_;
bool Profiler::enabled_ = false;

void Profiler::init() {
#ifdef ARE_ENABLE_PROFILING
    enabled_ = true;
    sections_.clear();
    results_.clear();
    ARE_LOG_INFO("Profiler initialized");
#else
    enabled_ = false;
    ARE_LOG_WARN("Profiler disabled (ARE_ENABLE_PROFILING not defined)");
#endif
}

void Profiler::shutdown() {
    if (!enabled_) {
        return;
    }

    print_results();
    sections_.clear();
    results_.clear();
    enabled_ = false;
}

void Profiler::begin(const std::string& name) {
    if (!enabled_) {
        return;
    }

    auto& section = sections_[name];
    section.start_time_ = std::chrono::high_resolution_clock::now();
}

void Profiler::end(const std::string& name) {
    if (!enabled_) {
        return;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    
    auto it = sections_.find(name);
    if (it == sections_.end()) {
        ARE_LOG_WARN("Profiler::end called for unknown section: " + name);
        return;
    }

    auto& section = it->second;
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        end_time - section.start_time_
    );
    
    section.total_duration_ms_ += duration.count() / 1000.0;
    section.call_count_++;
}

const std::unordered_map<std::string, ProfileResult>& Profiler::get_results() {
    if (!enabled_) {
        return results_;
    }

    results_.clear();
    
    for (const auto& [name, section] : sections_) {
        ProfileResult result;
        result.name_ = name;
        result.duration_ms_ = section.total_duration_ms_;
        result.call_count_ = section.call_count_;
        result.avg_duration_ms_ = (section.call_count_ > 0) 
            ? (section.total_duration_ms_ / section.call_count_) 
            : 0.0;
        
        results_[name] = result;
    }
    
    return results_;
}

void Profiler::reset() {
    if (!enabled_) {
        return;
    }

    for (auto& [name, section] : sections_) {
        section.total_duration_ms_ = 0.0;
        section.call_count_ = 0;
    }
    
    results_.clear();
}

void Profiler::print_results() {
    if (!enabled_) {
        return;
    }

    get_results();
    
    if (results_.empty()) {
        ARE_LOG_INFO("No profiling data available");
        return;
    }

    // Sort results by total duration (descending)
    std::vector<ProfileResult> sorted_results;
    sorted_results.reserve(results_.size());
    
    for (const auto& [name, result] : results_) {
        sorted_results.push_back(result);
    }
    
    std::sort(sorted_results.begin(), sorted_results.end(),
        [](const ProfileResult& a, const ProfileResult& b) {
            return a.duration_ms_ > b.duration_ms_;
        });

    // Print header
    ARE_LOG_INFO("=== Performance Profile ===");
    ARE_LOG_INFO(std::string(80, '-'));
    
    std::stringstream header;
    header << std::left << std::setw(30) << "Section"
           << std::right << std::setw(12) << "Total (ms)"
           << std::right << std::setw(12) << "Calls"
           << std::right << std::setw(12) << "Avg (ms)"
           << std::right << std::setw(12) << "Percent";
    ARE_LOG_INFO(header.str());
    ARE_LOG_INFO(std::string(80, '-'));

    // Calculate total time
    double total_time = 0.0;
    for (const auto& result : sorted_results) {
        total_time += result.duration_ms_;
    }

    // Print results
    for (const auto& result : sorted_results) {
        std::stringstream ss;
        double percent = (total_time > 0.0) ? (result.duration_ms_ / total_time * 100.0) : 0.0;
        
        ss << std::left << std::setw(30) << result.name_
           << std::right << std::setw(12) << std::fixed << std::setprecision(3) << result.duration_ms_
           << std::right << std::setw(12) << result.call_count_
           << std::right << std::setw(12) << std::fixed << std::setprecision(3) << result.avg_duration_ms_
           << std::right << std::setw(11) << std::fixed << std::setprecision(1) << percent << "%";
        
        ARE_LOG_INFO(ss.str());
    }

    ARE_LOG_INFO(std::string(80, '-'));
    
    std::stringstream total_ss;
    total_ss << "Total: " << std::fixed << std::setprecision(3) << total_time << " ms";
    ARE_LOG_INFO(total_ss.str());
    ARE_LOG_INFO("===========================");
}

// ScopedProfiler implementation
ScopedProfiler::ScopedProfiler(const std::string& name) : name_(name) {
    Profiler::begin(name_);
}

ScopedProfiler::~ScopedProfiler() {
    Profiler::end(name_);
}

} // namespace are
