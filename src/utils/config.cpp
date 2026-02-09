#include "utils/config.h"
#include "utils/logger.h"
#include <fstream>
#include <sstream>
#include <algorithm>

namespace are {

// Static storage
static std::unordered_map<std::string, std::string> g_config_map;

// Helper function to trim whitespace
static std::string trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    size_t last = str.find_last_not_of(" \t\r\n");
    return str.substr(first, last - first + 1);
}

bool Config::load(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        Logger::error("Failed to open config file: " + path);
        return false;
    }
    
    g_config_map.clear();
    
    std::string line;
    std::string current_section;
    
    while (std::getline(file, line)) {
        line = trim(line);
        
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#' || line[0] == ';') {
            continue;
        }
        
        // Section header
        if (line[0] == '[' && line.back() == ']') {
            current_section = line.substr(1, line.length() - 2);
            continue;
        }
        
        // Key-value pair
        size_t pos = line.find('=');
        if (pos != std::string::npos) {
            std::string key = trim(line.substr(0, pos));
            std::string value = trim(line.substr(pos + 1));
            
            // Add section prefix if in a section
            if (!current_section.empty()) {
                key = current_section + "." + key;
            }
            
            g_config_map[key] = value;
        }
    }
    
    Logger::info("Config loaded: " + path + " (" + std::to_string(g_config_map.size()) + " entries)");
    return true;
}

bool Config::save(const std::string& path) {
    std::ofstream file(path);
    if (!file.is_open()) {
        Logger::error("Failed to open config file for writing: " + path);
        return false;
    }
    
    for (const auto& pair : g_config_map) {
        file << pair.first << "=" << pair.second << std::endl;
    }
    
    Logger::info("Config saved: " + path);
    return true;
}

std::string Config::get_string(const std::string& key, const std::string& default_value) {
    auto it = g_config_map.find(key);
    if (it != g_config_map.end()) {
        return it->second;
    }
    return default_value;
}

int Config::get_int(const std::string& key, int default_value) {
    auto it = g_config_map.find(key);
    if (it != g_config_map.end()) {
        try {
            return std::stoi(it->second);
        } catch (...) {
            Logger::warning("Failed to parse int for key: " + key);
        }
    }
    return default_value;
}

float Config::get_float(const std::string& key, float default_value) {
    auto it = g_config_map.find(key);
    if (it != g_config_map.end()) {
        try {
            return std::stof(it->second);
        } catch (...) {
            Logger::warning("Failed to parse float for key: " + key);
        }
    }
    return default_value;
}

bool Config::get_bool(const std::string& key, bool default_value) {
    auto it = g_config_map.find(key);
    if (it != g_config_map.end()) {
        std::string value = it->second;
        std::transform(value.begin(), value.end(), value.begin(), ::tolower);
        
        if (value == "true" || value == "1" || value == "yes" || value == "on") {
            return true;
        }
        if (value == "false" || value == "0" || value == "no" || value == "off") {
            return false;
        }
    }
    return default_value;
}

void Config::set_string(const std::string& key, const std::string& value) {
    g_config_map[key] = value;
}

void Config::set_int(const std::string& key, int value) {
    g_config_map[key] = std::to_string(value);
}

void Config::set_float(const std::string& key, float value) {
    g_config_map[key] = std::to_string(value);
}

void Config::set_bool(const std::string& key, bool value) {
    g_config_map[key] = value ? "true" : "false";
}

} // namespace are
