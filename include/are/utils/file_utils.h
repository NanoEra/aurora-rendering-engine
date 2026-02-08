/**
 * @file file_utils.h
 * @brief File system utilities
 */

#ifndef ARE_INCLUDE_UTILS_FILE_UTILS_H
#define ARE_INCLUDE_UTILS_FILE_UTILS_H

#include <string>
#include <vector>
#include <cstdint>

namespace are {

/**
 * @brief Read entire file into string
 * @param filepath File path
 * @return File contents (empty if failed)
 */
std::string read_file_to_string(const std::string& filepath);

/**
 * @brief Read entire file into byte array
 * @param filepath File path
 * @return File contents (empty if failed)
 */
std::vector<uint8_t> read_file_to_bytes(const std::string& filepath);

/**
 * @brief Write string to file
 * @param filepath File path
 * @param content Content to write
 * @return true if write succeeded
 */
bool write_string_to_file(const std::string& filepath, const std::string& content);

/**
 * @brief Write bytes to file
 * @param filepath File path
 * @param data Data pointer
 * @param size Data size in bytes
 * @return true if write succeeded
 */
bool write_bytes_to_file(const std::string& filepath, const void* data, size_t size);

/**
 * @brief Check if file exists
 * @param filepath File path
 * @return true if file exists
 */
bool file_exists(const std::string& filepath);

/**
 * @brief Check if path is directory
 * @param path Directory path
 * @return true if directory exists
 */
bool is_directory(const std::string& path);

/**
 * @brief Create directory (including parent directories)
 * @param path Directory path
 * @return true if creation succeeded
 */
bool create_directory(const std::string& path);

/**
 * @brief Get file extension
 * @param filepath File path
 * @return Extension (lowercase, without dot)
 */
std::string get_file_extension(const std::string& filepath);

/**
 * @brief Get filename from path
 * @param filepath File path
 * @return Filename (without directory)
 */
std::string get_filename(const std::string& filepath);

/**
 * @brief Get directory from path
 * @param filepath File path
 * @return Directory path
 */
std::string get_directory(const std::string& filepath);

/**
 * @brief Join path components
 * @param parts Path components
 * @return Joined path
 */
std::string join_path(const std::vector<std::string>& parts);

/**
 * @brief Normalize path (resolve .. and .)
 * @param path Path to normalize
 * @return Normalized path
 */
std::string normalize_path(const std::string& path);

} // namespace are

#endif // ARE_INCLUDE_UTILS_FILE_UTILS_H
