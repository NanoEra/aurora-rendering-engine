/**
 * @file file_utils.cpp
 * @brief Implementation of file system utilities
 */

#include <are/utils/file_utils.h>
#include <are/core/logger.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <sys/stat.h>

#ifdef _WIN32
    #include <direct.h>
    #define MKDIR(path) _mkdir(path)
#else
    #include <sys/types.h>
    #define MKDIR(path) mkdir(path, 0755)
#endif

namespace are {

std::string read_file_to_string(const std::string& filepath) {
    std::ifstream file(filepath, std::ios::in | std::ios::binary);
    
    if (!file.is_open()) {
        ARE_LOG_ERROR("Failed to open file: " + filepath);
        return "";
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();
    
    return buffer.str();
}

std::vector<uint8_t> read_file_to_bytes(const std::string& filepath) {
    std::ifstream file(filepath, std::ios::in | std::ios::binary);
    
    if (!file.is_open()) {
        ARE_LOG_ERROR("Failed to open file: " + filepath);
        return {};
    }

    // Get file size
    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);

    // Read data
    std::vector<uint8_t> data(size);
    file.read(reinterpret_cast<char*>(data.data()), size);
    file.close();

    return data;
}

bool write_string_to_file(const std::string& filepath, const std::string& content) {
    std::ofstream file(filepath, std::ios::out | std::ios::binary);
    
    if (!file.is_open()) {
        ARE_LOG_ERROR("Failed to open file for writing: " + filepath);
        return false;
    }

    file << content;
    file.close();
    
    return true;
}

bool write_bytes_to_file(const std::string& filepath, const void* data, size_t size) {
    std::ofstream file(filepath, std::ios::out | std::ios::binary);
    
    if (!file.is_open()) {
        ARE_LOG_ERROR("Failed to open file for writing: " + filepath);
        return false;
    }

    file.write(static_cast<const char*>(data), size);
    file.close();
    
    return true;
}

bool file_exists(const std::string& filepath) {
    struct stat buffer;
    return (stat(filepath.c_str(), &buffer) == 0);
}

bool is_directory(const std::string& path) {
    struct stat buffer;
    if (stat(path.c_str(), &buffer) != 0) {
        return false;
    }
    return (buffer.st_mode & S_IFDIR) != 0;
}

bool create_directory(const std::string& path) {
    if (path.empty()) {
        return false;
    }

    if (is_directory(path)) {
        return true;
    }

    // Create parent directories recursively
    size_t pos = 0;
    std::string current_path;
    
    while ((pos = path.find_first_of("/\\", pos)) != std::string::npos) {
        current_path = path.substr(0, pos++);
        
        if (!current_path.empty() && !is_directory(current_path)) {
            if (MKDIR(current_path.c_str()) != 0 && !is_directory(current_path)) {
                ARE_LOG_ERROR("Failed to create directory: " + current_path);
                return false;
            }
        }
    }

    // Create final directory
    if (MKDIR(path.c_str()) != 0 && !is_directory(path)) {
        ARE_LOG_ERROR("Failed to create directory: " + path);
        return false;
    }

    return true;
}

std::string get_file_extension(const std::string& filepath) {
    size_t dot_pos = filepath.find_last_of('.');
    
    if (dot_pos == std::string::npos || dot_pos == filepath.length() - 1) {
        return "";
    }

    std::string ext = filepath.substr(dot_pos + 1);
    
    // Convert to lowercase
    std::transform(ext.begin(), ext.end(), ext.begin(), 
                  [](unsigned char c) { return std::tolower(c); });
    
    return ext;
}

std::string get_filename(const std::string& filepath) {
    size_t slash_pos = filepath.find_last_of("/\\");
    
    if (slash_pos == std::string::npos) {
        return filepath;
    }

    return filepath.substr(slash_pos + 1);
}

std::string get_directory(const std::string& filepath) {
    size_t slash_pos = filepath.find_last_of("/\\");
    
    if (slash_pos == std::string::npos) {
        return "";
    }

    return filepath.substr(0, slash_pos);
}

std::string join_path(const std::vector<std::string>& parts) {
    if (parts.empty()) {
        return "";
    }

    std::string result = parts[0];
    
    for (size_t i = 1; i < parts.size(); ++i) {
        if (!result.empty() && result.back() != '/' && result.back() != '\\') {
            result += '/';
        }
        result += parts[i];
    }

    return result;
}

std::string normalize_path(const std::string& path) {
    if (path.empty()) {
        return "";
    }

    std::vector<std::string> components;
    std::string current;
    
    for (char c : path) {
        if (c == '/' || c == '\\') {
            if (!current.empty()) {
                if (current == "..") {
                    if (!components.empty() && components.back() != "..") {
                        components.pop_back();
                    } else {
                        components.push_back(current);
                    }
                } else if (current != ".") {
                    components.push_back(current);
                }
                current.clear();
            }
        } else {
            current += c;
        }
    }

    if (!current.empty()) {
        if (current == "..") {
            if (!components.empty() && components.back() != "..") {
                components.pop_back();
            } else {
                components.push_back(current);
            }
        } else if (current != ".") {
            components.push_back(current);
        }
    }

    return join_path(components);
}

} // namespace are
