/**
 * @file image_io.h
 * @brief Image loading and saving utilities
 */

#ifndef ARE_INCLUDE_UTILS_IMAGE_IO_H
#define ARE_INCLUDE_UTILS_IMAGE_IO_H

#include <are/core/types.h>
#include <string>
#include <vector>

namespace are {

/**
 * @enum ImageFormat
 * @brief Supported image formats
 */
enum class ImageFormat {
    ARE_IMAGE_FORMAT_PPM,
    ARE_IMAGE_FORMAT_BMP,
    ARE_IMAGE_FORMAT_PNG,
    ARE_IMAGE_FORMAT_JPG
};

/**
 * @struct ImageData
 * @brief Container for image data
 */
struct ImageData {
    int width_;                           ///< Image width
    int height_;                          ///< Image height
    int channels_;                        ///< Number of channels (3=RGB, 4=RGBA)
    std::vector<uint8_t> data_;           ///< Pixel data (row-major)

    /**
     * @brief Check if image data is valid
     * @return true if valid
     */
    bool is_valid() const;

    /**
     * @brief Get pixel at (x, y)
     * @param x X coordinate
     * @param y Y coordinate
     * @return Pointer to pixel data (RGB or RGBA)
     */
    const uint8_t* get_pixel(int x, int y) const;

    /**
     * @brief Set pixel at (x, y)
     * @param x X coordinate
     * @param y Y coordinate
     * @param r Red channel
     * @param g Green channel
     * @param b Blue channel
     * @param a Alpha channel (optional)
     */
    void set_pixel(int x, int y, uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255);
};

/**
 * @brief Load image from file
 * @param filename Image file path
 * @param flip_vertically Whether to flip image vertically
 * @return Image data (empty if failed)
 */
ImageData load_image(const std::string& filename, bool flip_vertically = false);

/**
 * @brief Save image to file
 * @param filename Output file path
 * @param data Image data
 * @param format Output format (auto-detected from extension if not specified)
 * @return true if save succeeded
 */
bool save_image(const std::string& filename, const ImageData& data, 
               ImageFormat format = ImageFormat::ARE_IMAGE_FORMAT_PNG);

/**
 * @brief Save raw pixel data to file
 * @param filename Output file path
 * @param pixels Pixel data pointer
 * @param width Image width
 * @param height Image height
 * @param channels Number of channels
 * @param format Output format
 * @return true if save succeeded
 */
bool save_image(const std::string& filename, const uint8_t* pixels,
               int width, int height, int channels,
               ImageFormat format = ImageFormat::ARE_IMAGE_FORMAT_PNG);

/**
 * @brief Detect image format from file extension
 * @param filename File path
 * @return Detected format
 */
ImageFormat detect_format(const std::string& filename);

} // namespace are

#endif // ARE_INCLUDE_UTILS_IMAGE_IO_H
