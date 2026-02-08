/**
 * @file image_io.cpp
 * @brief Implementation of image I/O utilities
 */

#include <are/utils/image_io.h>
#include <are/core/logger.h>
#include <cstring>
#include <algorithm>
#include <string>
#include <fstream>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

namespace are {

bool ImageData::is_valid() const {
    return width_ > 0 && height_ > 0 && channels_ > 0 && !data_.empty();
}

const uint8_t* ImageData::get_pixel(int x, int y) const {
    if (x < 0 || x >= width_ || y < 0 || y >= height_) {
        return nullptr;
    }
    
    size_t index = (y * width_ + x) * channels_;
    return &data_[index];
}

void ImageData::set_pixel(int x, int y, uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
    if (x < 0 || x >= width_ || y < 0 || y >= height_) {
        return;
    }
    
    size_t index = (y * width_ + x) * channels_;
    
    data_[index + 0] = r;
    data_[index + 1] = g;
    data_[index + 2] = b;
    
    if (channels_ == 4) {
        data_[index + 3] = a;
    }
}

ImageData load_image(const std::string& filename, bool flip_vertically) {
    ImageData image;
    
    // Set flip flag
    stbi_set_flip_vertically_on_load(flip_vertically ? 1 : 0);
    
    // Load image
    int width, height, channels;
    unsigned char* data = stbi_load(filename.c_str(), &width, &height, &channels, 0);
    
    if (!data) {
        ARE_LOG_ERROR("Failed to load image: " + filename + " - " + stbi_failure_reason());
        return image;
    }
    
    // Copy data to ImageData
    image.width_ = width;
    image.height_ = height;
    image.channels_ = channels;
    
    size_t data_size = width * height * channels;
    image.data_.resize(data_size);
    std::memcpy(image.data_.data(), data, data_size);
    
    // Free stb_image data
    stbi_image_free(data);
    
    ARE_LOG_INFO("Loaded image: " + filename + " (" + 
                 std::to_string(width) + "x" + std::to_string(height) + 
                 ", " + std::to_string(channels) + " channels)");
    
    return image;
}

ImageFormat detect_format(const std::string& filename) {
    std::string ext;
    size_t dot_pos = filename.find_last_of('.');
    
    if (dot_pos != std::string::npos) {
        ext = filename.substr(dot_pos + 1);
        
        // Convert to lowercase
        std::transform(ext.begin(), ext.end(), ext.begin(),
                      [](unsigned char c) { return std::tolower(c); });
    }
    
    if (ext == "ppm") return ImageFormat::ARE_IMAGE_FORMAT_PPM;
    if (ext == "bmp") return ImageFormat::ARE_IMAGE_FORMAT_BMP;
    if (ext == "png") return ImageFormat::ARE_IMAGE_FORMAT_PNG;
    if (ext == "jpg" || ext == "jpeg") return ImageFormat::ARE_IMAGE_FORMAT_JPG;
    
    // Default to PNG
    return ImageFormat::ARE_IMAGE_FORMAT_PNG;
}

bool save_image(const std::string& filename, const ImageData& data, ImageFormat format) {
    if (!data.is_valid()) {
        ARE_LOG_ERROR("Cannot save invalid image data");
        return false;
    }
    
    // Auto-detect format if not specified
    if (format == ImageFormat::ARE_IMAGE_FORMAT_PNG) {
        format = detect_format(filename);
    }
    
    int result = 0;
    
    switch (format) {
        case ImageFormat::ARE_IMAGE_FORMAT_PPM: {
            // Write PPM manually (stb doesn't support it)
            std::ofstream file(filename, std::ios::binary);
            if (!file.is_open()) {
                ARE_LOG_ERROR("Failed to open file for writing: " + filename);
                return false;
            }
            
            file << "P6\n" << data.width_ << " " << data.height_ << "\n255\n";
            
            for (int i = 0; i < data.width_ * data.height_; ++i) {
                int idx = i * data.channels_;
                file.put(data.data_[idx + 0]); // R
                file.put(data.data_[idx + 1]); // G
                file.put(data.data_[idx + 2]); // B
            }
            
            file.close();
            result = 1;
            break;
        }
        
        case ImageFormat::ARE_IMAGE_FORMAT_BMP:
            result = stbi_write_bmp(filename.c_str(), data.width_, data.height_, 
                                   data.channels_, data.data_.data());
            break;
        
        case ImageFormat::ARE_IMAGE_FORMAT_PNG:
            result = stbi_write_png(filename.c_str(), data.width_, data.height_, 
                                   data.channels_, data.data_.data(), 
                                   data.width_ * data.channels_);
            break;
        
        case ImageFormat::ARE_IMAGE_FORMAT_JPG:
            result = stbi_write_jpg(filename.c_str(), data.width_, data.height_, 
                                   data.channels_, data.data_.data(), 90);
            break;
    }
    
    if (result == 0) {
        ARE_LOG_ERROR("Failed to save image: " + filename);
        return false;
    }
    
    ARE_LOG_INFO("Saved image: " + filename);
    return true;
}

bool save_image(const std::string& filename, const uint8_t* pixels,
               int width, int height, int channels, ImageFormat format) {
    ImageData data;
    data.width_ = width;
    data.height_ = height;
    data.channels_ = channels;
    
    size_t data_size = width * height * channels;
    data.data_.resize(data_size);
    std::memcpy(data.data_.data(), pixels, data_size);
    
    return save_image(filename, data, format);
}

} // namespace are
