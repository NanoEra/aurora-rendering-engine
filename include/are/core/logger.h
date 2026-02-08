/**
 * @file logger.h
 * @brief Logging system for the rendering engine
 */

#ifndef ARE_INCLUDE_CORE_LOGGER_H
#define ARE_INCLUDE_CORE_LOGGER_H

#include <string>
#include <memory>

namespace are {

/**
 * @enum LogLevel
 * @brief Logging severity levels
 */
enum class LogLevel {
    ARE_LOG_TRACE,
    ARE_LOG_DEBUG,
    ARE_LOG_INFO,
    ARE_LOG_WARN,
    ARE_LOG_ERROR,
    ARE_LOG_CRITICAL
};

/**
 * @class Logger
 * @brief Thread-safe logging system
 * 
 * This class provides a simple interface for logging messages with different
 * severity levels. It wraps spdlog for actual logging functionality.
 */
class Logger {
public:
    /**
     * @brief Initialize the logging system
     * @param min_level Minimum log level to display
     */
    static void init(LogLevel min_level = LogLevel::ARE_LOG_INFO);

    /**
     * @brief Shutdown the logging system
     */
    static void shutdown();

    /**
     * @brief Log a message with file/function/line information
     * @param level Log severity level
     * @param file Source file name
     * @param func Function name
     * @param line Line number
     * @param message Log message
     */
    static void log(LogLevel level, const char* file, const char* func, 
                   int line, const std::string& message);

    /**
     * @brief Set minimum log level
     * @param level Minimum log level to display
     */
    static void set_level(LogLevel level);

private:
    static std::shared_ptr<void> logger_impl_; ///< Internal logger implementation
    static bool initialized_;                   ///< Initialization flag
};

} // namespace are

// Logging macros
#define ARE_LOG_TRACE(msg)    are::Logger::log(are::LogLevel::ARE_LOG_TRACE, __FILE__, __func__, __LINE__, msg)
#define ARE_LOG_DEBUG(msg)    are::Logger::log(are::LogLevel::ARE_LOG_DEBUG, __FILE__, __func__, __LINE__, msg)
#define ARE_LOG_INFO(msg)     are::Logger::log(are::LogLevel::ARE_LOG_INFO, __FILE__, __func__, __LINE__, msg)
#define ARE_LOG_WARN(msg)     are::Logger::log(are::LogLevel::ARE_LOG_WARN, __FILE__, __func__, __LINE__, msg)
#define ARE_LOG_ERROR(msg)    are::Logger::log(are::LogLevel::ARE_LOG_ERROR, __FILE__, __func__, __LINE__, msg)
#define ARE_LOG_CRITICAL(msg) are::Logger::log(are::LogLevel::ARE_LOG_CRITICAL, __FILE__, __func__, __LINE__, msg)

#endif // ARE_INCLUDE_CORE_LOGGER_H
