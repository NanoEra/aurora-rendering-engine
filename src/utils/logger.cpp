#include "utils/logger.h"
#include <iostream>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace are {

// Static members
static LogLevel g_min_level = LogLevel::DEBUG;
static std::ofstream g_log_file;
static bool g_initialized = false;

bool Logger::initialize(const std::string& log_file) {
    if (g_initialized) {
        return true;
    }
    
    if (!log_file.empty()) {
        g_log_file.open(log_file, std::ios::out | std::ios::app);
        if (!g_log_file.is_open()) {
            std::cerr << "Failed to open log file: " << log_file << std::endl;
            return false;
        }
    }
    
    g_initialized = true;
    return true;
}

void Logger::shutdown() {
    if (g_log_file.is_open()) {
        g_log_file.close();
    }
    g_initialized = false;
}

static std::string get_current_time() {
    auto now = std::time(nullptr);
    auto tm = *std::localtime(&now);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%H:%M:%S");
    return oss.str();
}

static std::string level_to_string(LogLevel level) {
    switch (level) {
        case LogLevel::DEBUG: return "DEBUG";
        case LogLevel::INFO: return "INFO";
        case LogLevel::WARNING: return "WARN";
        case LogLevel::ERROR: return "ERROR";
        case LogLevel::FATAL: return "FATAL";
        default: return "UNKNOWN";
    }
}

void Logger::log(LogLevel level, const std::string& message) {
    if (level < g_min_level) return;
    
    std::string time_str = get_current_time();
    std::string level_str = level_to_string(level);
    std::string formatted = "[" + time_str + "] [" + level_str + "] " + message;
    
    // Console output
    if (level >= LogLevel::ERROR) {
        std::cerr << formatted << std::endl;
    } else {
        std::cout << formatted << std::endl;
    }
    
    // File output
    if (g_log_file.is_open()) {
        g_log_file << formatted << std::endl;
        g_log_file.flush();
    }
}

void Logger::debug(const std::string& message) {
    log(LogLevel::DEBUG, message);
}

void Logger::info(const std::string& message) {
    log(LogLevel::INFO, message);
}

void Logger::warning(const std::string& message) {
    log(LogLevel::WARNING, message);
}

void Logger::error(const std::string& message) {
    log(LogLevel::ERROR, message);
}

void Logger::fatal(const std::string& message) {
    log(LogLevel::FATAL, message);
}

void Logger::set_level(LogLevel level) {
    g_min_level = level;
}

} // namespace are
