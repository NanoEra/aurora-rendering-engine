#include "utils/logger.h"
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <mutex>
#include <cstring>

namespace are {

std::shared_ptr<void> Logger::logger_impl_ = nullptr;
bool Logger::initialized_ = false;

void Logger::init(LogLevel min_level) {
    if (initialized_) {
        return;
    }

    try {
        // Create console sink with color support
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");

        // Create logger with console sink
        auto logger = std::make_shared<spdlog::logger>("are", console_sink);
        
        // Set log level
        switch (min_level) {
            case LogLevel::ARE_LOG_TRACE:    
                logger->set_level(spdlog::level::trace); 
                break;
            case LogLevel::ARE_LOG_DEBUG:    
                logger->set_level(spdlog::level::debug); 
                break;
            case LogLevel::ARE_LOG_INFO:     
                logger->set_level(spdlog::level::info); 
                break;
            case LogLevel::ARE_LOG_WARN:     
                logger->set_level(spdlog::level::warn); 
                break;
            case LogLevel::ARE_LOG_ERROR:    
                logger->set_level(spdlog::level::err); 
                break;
            case LogLevel::ARE_LOG_CRITICAL: 
                logger->set_level(spdlog::level::critical); 
                break;
        }

        // Flush on error or higher
        logger->flush_on(spdlog::level::err);

        // Set as default logger
        spdlog::set_default_logger(logger);
        logger_impl_ = logger;
        initialized_ = true;

    } catch (const std::exception& e) {
        fprintf(stderr, "[ARE] Failed to initialize logger: %s\n", e.what());
    }
}

void Logger::shutdown() {
    if (!initialized_) {
        return;
    }

    try {
        spdlog::shutdown();
        logger_impl_.reset();
        initialized_ = false;
    } catch (const std::exception& e) {
        fprintf(stderr, "[ARE] Error during logger shutdown: %s\n", e.what());
    }
}

void Logger::log(LogLevel level, const char* file, const char* func, 
                int line, const std::string& message) {
    if (!initialized_) {
        init();
    }

    // Extract filename from full path
    const char* filename = file;
    const char* last_slash = nullptr;
    
    for (const char* p = file; *p; ++p) {
        if (*p == '/' || *p == '\\') {
            last_slash = p;
        }
    }
    
    if (last_slash) {
        filename = last_slash + 1;
    }

    // Format message with location information
    std::string formatted = message + " (" + filename + "-" + func + "():" + std::to_string(line) + ")";

    try {
        auto logger = std::static_pointer_cast<spdlog::logger>(logger_impl_);
        
        switch (level) {
            case LogLevel::ARE_LOG_TRACE:
                logger->trace(formatted);
                break;
            case LogLevel::ARE_LOG_DEBUG:
                logger->debug(formatted);
                break;
            case LogLevel::ARE_LOG_INFO:
                logger->info(formatted);
                break;
            case LogLevel::ARE_LOG_WARN:
                logger->warn(formatted);
                break;
            case LogLevel::ARE_LOG_ERROR:
                logger->error(formatted);
                break;
            case LogLevel::ARE_LOG_CRITICAL:
                logger->critical(formatted);
                break;
        }
    } catch (const std::exception& e) {
        fprintf(stderr, "[ARE] Logging error: %s\n", e.what());
    }
}

void Logger::set_level(LogLevel level) {
    if (!initialized_) {
        return;
    }

    try {
        auto logger = std::static_pointer_cast<spdlog::logger>(logger_impl_);
        
        switch (level) {
            case LogLevel::ARE_LOG_TRACE:    
                logger->set_level(spdlog::level::trace); 
                break;
            case LogLevel::ARE_LOG_DEBUG:    
                logger->set_level(spdlog::level::debug); 
                break;
            case LogLevel::ARE_LOG_INFO:     
                logger->set_level(spdlog::level::info); 
                break;
            case LogLevel::ARE_LOG_WARN:     
                logger->set_level(spdlog::level::warn); 
                break;
            case LogLevel::ARE_LOG_ERROR:    
                logger->set_level(spdlog::level::err); 
                break;
            case LogLevel::ARE_LOG_CRITICAL: 
                logger->set_level(spdlog::level::critical); 
                break;
        }
    } catch (const std::exception& e) {
        fprintf(stderr, "[ARE] Error setting log level: %s\n", e.what());
    }
}

} // namespace are
