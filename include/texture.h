#ifndef ARE_INCLUDE_TEXTURE_H
#define ARE_INCLUDE_TEXTURE_H

#include <basic/vec3.h>
#include <string>
#include <utility>

namespace are {

class Texture {
public:
	// Constructors
	Texture() = default; // Allow default constructor.
	Texture(const std::string &image_path); // Initialize the image by loading from file (currently support .ppm format).
	Texture(int width, int height, const Color3 &fill_color); // Initialize the image by filling with a color.

	// Destructor
	~Texture();

	Color3 &pixel(int x, int y);

	// Paste src_texture into this image with given four corners.
	void paste(const Texture &src_texture, const std::pair<int, int> &left_top, const std::pair<int, int> &right_top, const std::pair<int, int> &left_bottom, const std::pair<int, int> &right_bottom);

	// Save the texture to a file (currently support .ppm format)
	bool save_texture(const std::string &file_path) const;

	int width_, height_; // The image width and height.

private:
	Color3 **image_ = nullptr; // The raw pixel data.
};

}

#endif
