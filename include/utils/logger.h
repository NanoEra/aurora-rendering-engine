#ifndef ARE_INCLUDE_UTILS_LOGGER_H
#define ARE_INCLUDE_UTILS_LOGGER_H

#include <string>

namespace are {

/// @brief Log level enumeration
enum class LogLevel {
    DEBUG,
    INFO,
    WARNING,
    ERROR,
    FATAL
};

/// @brief Logger interface for engine logging
/// @note This module should be implemented by the user
class Logger {
public:
    /// @brief Initialize logger
    /// @param log_file Log file path (empty for console only)
    /// @return True if initialization succeeded
    static bool initialize(const std::string& log_file = "");
    
    /// @brief Shutdown logger
    static void shutdown();
    
    /// @brief Log message
    /// @param level Log level
    /// @param message Message content
    static void log(LogLevel level, const std::string& message);
    
    /// @brief Log debug message
    /// @param message Message content
    static void debug(const std::string& message);
    
    /// @brief Log info message
    /// @param message Message content
    static void info(const std::string& message);
    
    /// @brief Log warning message
    /// @param message Message content
    static void warning(const std::string& message);
    
    /// @brief Log error message
    /// @param message Message content
    static void error(const std::string& message);
    
    /// @brief Log fatal message
    /// @param message Message content
    static void fatal(const std::string& message);
    
    /// @brief Set minimum log level
    /// @param level Minimum level to log
    static void set_level(LogLevel level);
};

} // namespace are

#endif // ARE_INCLUDE_UTILS_LOGGER_H
