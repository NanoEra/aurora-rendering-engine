#ifndef ARE_INCLUDE_UTILS_CONFIG_H
#define ARE_INCLUDE_UTILS_CONFIG_H

#include <string>
#include <unordered_map>

namespace are {

/// @brief Configuration manager for loading engine settings
/// @note This module should be implemented by the user
class Config {
public:
    /// @brief Load configuration from file
    /// @param path Configuration file path
    /// @return True if loading succeeded
    static bool load(const std::string& path);
    
    /// @brief Save configuration to file
    /// @param path Configuration file path
    /// @return True if saving succeeded
    static bool save(const std::string& path);
    
    /// @brief Get string value
    /// @param key Configuration key
    /// @param default_value Default value if key not found
    /// @return Configuration value
    static std::string get_string(const std::string& key, const std::string& default_value = "");
    
    /// @brief Get integer value
    /// @param key Configuration key
    /// @param default_value Default value if key not found
    /// @return Configuration value
    static int get_int(const std::string& key, int default_value = 0);
    
    /// @brief Get float value
    /// @param key Configuration key
    /// @param default_value Default value if key not found
    /// @return Configuration value
    static float get_float(const std::string& key, float default_value = 0.0f);
    
    /// @brief Get boolean value
    /// @param key Configuration key
    /// @param default_value Default value if key not found
    /// @return Configuration value
    static bool get_bool(const std::string& key, bool default_value = false);
    
    /// @brief Set string value
    /// @param key Configuration key
    /// @param value Configuration value
    static void set_string(const std::string& key, const std::string& value);
    
    /// @brief Set integer value
    /// @param key Configuration key
    /// @param value Configuration value
    static void set_int(const std::string& key, int value);
    
    /// @brief Set float value
    /// @param key Configuration key
    /// @param value Configuration value
    static void set_float(const std::string& key, float value);
    
    /// @brief Set boolean value
    /// @param key Configuration key
    /// @param value Configuration value
    static void set_bool(const std::string& key, bool value);
};

} // namespace are

#endif // ARE_INCLUDE_UTILS_CONFIG_H
