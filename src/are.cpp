/**
 * @file are.cpp
 * @brief Implementation of main engine interface
 */

#include <are/are.h>
#include <are/core/logger.h>
#include <are/core/profiler.h>

namespace are {

const char* get_version() {
    return "0.1.0";
}

bool initialize() {
    // Initialize logger first
    Logger::init(LogLevel::ARE_LOG_INFO);
    
    ARE_LOG_INFO("===========================================");
    ARE_LOG_INFO("Aurora Rendering Engine v" + std::string(get_version()));
    ARE_LOG_INFO("===========================================");
    ARE_LOG_INFO("Initializing engine...");
    
    // Initialize profiler
    Profiler::init();
    
    ARE_LOG_INFO("Engine initialization complete");
    return true;
}

void shutdown() {
    ARE_LOG_INFO("Shutting down Aurora Rendering Engine...");
    
    // Shutdown profiler
    Profiler::shutdown();
    
    ARE_LOG_INFO("Engine shutdown complete");
    
    // Shutdown logger last
    Logger::shutdown();
}

} // namespace are
