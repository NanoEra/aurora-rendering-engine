#include <algorithm>
#include <basic/math.h>
#include <stdexcept>
#include <string>
#include <texture.h>

namespace are {

Texture::Texture(const std::string &image_path) {

	FILE *fp = fopen(image_path.c_str(), "rb");
	if (!fp) {
		throw std::runtime_error("Failed to open texture file: " + image_path);
	}

	// Read PPM header
	char format[3];
	if (fscanf(fp, "%2s", format) != 1 || std::string(format) != "P6") {
		fclose(fp);
		throw std::runtime_error("Unsupported PPM format in file: " + image_path);
	}
	int width, height, max_val;
	if (fscanf(fp, "%d %d %d", &width, &height, &max_val) != 3 || max_val != 255) {
		fclose(fp);
		throw std::runtime_error("Invalid PPM header in file: " + image_path);
	}
	fgetc(fp); // Consume the newline after max_val
	width_ = width, height_ = height; // Set texture dimensions

	// Allocate the memory for the pixel data.
	// [WARN]: In order to ensure the cache hit rate(we tend to traverse the pixel array by line), the image_ array is allocated with declaration image_[height][width], it means you should access the array by image[y][x].
	image_ = new Color3 *[height];
	for (int i = 0; i < height; image_[i] = new Color3[width], ++i)
		;

	// Read pixel data
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			unsigned char r, g, b;
			if (fread(&r, sizeof(unsigned char), 1, fp) != 1 || fread(&g, sizeof(unsigned char), 1, fp) != 1 || fread(&b, sizeof(unsigned char), 1, fp) != 1) {
				fclose(fp);
				throw std::runtime_error("Unexpected end of file while reading pixel data in file: " + image_path);
			}
			image_[y][x] = Color3(static_cast<double>(r) / 255.0,
				static_cast<double>(g) / 255.0,
				static_cast<double>(b) / 255.0);
		}
	}
	fclose(fp);
}

Texture::Texture(int width, int height, const Color3 &fill_color) : width_(width), height_(height) {
	if (width <= 0 || height <= 0) {
		throw std::runtime_error("Texture width and height must be positive.");
	}

	// Allocate the memory for the pixel data.
	// [WARN]: In order to ensure the cache hit rate(we tend to traverse the pixel array by line), the image_ array is allocated with declaration image_[height][width], it means you should access the array by image[y][x].
	image_ = new Color3 *[height];
	for (int i = 0; i < height; ++i) {
		image_[i] = new Color3[width];
		// Fill the image with the specified color.
		for (int j = 0; j < width; image_[i][j] = fill_color, ++j)
			;
	}
}

Texture::~Texture() {
	if (!image_) {
		return;
	}

	for (int i = 0; i < height_; delete[] image_[i], ++i)
		;
}

Color3 &Texture::pixel(int x, int y) {
	if (!image_) {
		throw std::runtime_error("Texture is not initialized.");
	}

	return image_[y][x];
}

void Texture::paste(const Texture &src_image,
	const std::pair<int, int> &left_top,
	const std::pair<int, int> &right_top,
	const std::pair<int, int> &left_bottom,
	const std::pair<int, int> &right_bottom) {
	if (!image_) {
		throw std::runtime_error("Texture is not initialized.");
	}
	if (src_image.width_ <= 0 || src_image.height_ <= 0) {
		return;
	}

	struct Pt {
		double x, y;
	};

	auto toPt = [](const std::pair<int, int> &p) -> Pt {
		return Pt {
			static_cast<double>(p.first),
			static_cast<double>(p.second)
		};
	};

	// Destination quad vertices (consistent order for polygon test)
	// Note: Given points are LT, RT, LB, RB, but polygon order should be around the boundary.
	// We'll use: LT -> RT -> RB -> LB (clockwise / counterclockwise both ok for ray casting).
	Pt quad[4] = {
		toPt(left_top),
		toPt(right_top),
		toPt(right_bottom),
		toPt(left_bottom),
	};

	// Compute destination bounding box (in double), then clip to destination image bounds.
	double min_x_d = std::min({ quad[0].x, quad[1].x, quad[2].x, quad[3].x });
	double max_x_d = std::max({ quad[0].x, quad[1].x, quad[2].x, quad[3].x });
	double min_y_d = std::min({ quad[0].y, quad[1].y, quad[2].y, quad[3].y });
	double max_y_d = std::max({ quad[0].y, quad[1].y, quad[2].y, quad[3].y });

	// If quad's bbox is completely outside destination, nothing to do (robustness).
	if (max_x_d < 0.0 || max_y_d < 0.0 || min_x_d > static_cast<double>(width_ - 1) || min_y_d > static_cast<double>(height_ - 1)) {
		return;
	}

	int x0 = std::max(0, static_cast<int>(std::floor(min_x_d)));
	int x1 = std::min(width_ - 1, static_cast<int>(std::ceil(max_x_d)));
	int y0 = std::max(0, static_cast<int>(std::floor(min_y_d)));
	int y1 = std::min(height_ - 1, static_cast<int>(std::ceil(max_y_d)));

	// Point in polygon test (ray casting, works for convex/concave simple polygons)
	auto pointInPoly = [](double x, double y, const Pt *poly, int n) -> bool {
		bool inside = false;
		for (int i = 0, j = n - 1; i < n; j = i++) {
			const double xi = poly[i].x, yi = poly[i].y;
			const double xj = poly[j].x, yj = poly[j].y;

			const bool intersect = ((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / ((yj - yi) == 0.0 ? 1e-30 : (yj - yi)) + xi);
			inside ^= intersect; // if (intersect) inside = !inside;
		}
		return inside;
	};

	// Build homography from src rectangle corners to dest quad corners.
	// Src corners in pixel coordinates:
	// (0,0) -> left_top
	// (w-1,0) -> right_top
	// (0,h-1) -> left_bottom
	// (w-1,h-1) -> right_bottom
	const double sw = static_cast<double>(src_image.width_);
	const double sh = static_cast<double>(src_image.height_);
	const double sx0 = 0.0, sy0 = 0.0;
	const double sx1 = sw - 1.0, sy1 = 0.0;
	const double sx2 = 0.0, sy2 = sh - 1.0;
	const double sx3 = sw - 1.0, sy3 = sh - 1.0;

	const Pt dst0 = toPt(left_top);
	const Pt dst1 = toPt(right_top);
	const Pt dst2 = toPt(left_bottom);
	const Pt dst3 = toPt(right_bottom);

	// Solve A * h = b, where h = [h11 h12 h13 h21 h22 h23 h31 h32]^T, and h33 = 1
	double A[8][9] = {}; // augmented matrix [A|b]

	auto setEquations = [&](int row, double x, double y, double u, double v) {
		// x' = (h11 x + h12 y + h13) / (h31 x + h32 y + 1)
		// y' = (h21 x + h22 y + h23) / (h31 x + h32 y + 1)
		//
		// => h11 x + h12 y + h13 - u h31 x - u h32 y = u
		// => h21 x + h22 y + h23 - v h31 x - v h32 y = v
		//
		// Row for x':
		A[row + 0][0] = x;
		A[row + 0][1] = y;
		A[row + 0][2] = 1.0;
		A[row + 0][3] = 0.0;
		A[row + 0][4] = 0.0;
		A[row + 0][5] = 0.0;
		A[row + 0][6] = -u * x;
		A[row + 0][7] = -u * y;
		A[row + 0][8] = u;

		// Row for y':
		A[row + 1][0] = 0.0;
		A[row + 1][1] = 0.0;
		A[row + 1][2] = 0.0;
		A[row + 1][3] = x;
		A[row + 1][4] = y;
		A[row + 1][5] = 1.0;
		A[row + 1][6] = -v * x;
		A[row + 1][7] = -v * y;
		A[row + 1][8] = v;
	};

	setEquations(0, sx0, sy0, dst0.x, dst0.y);
	setEquations(2, sx1, sy1, dst1.x, dst1.y);
	setEquations(4, sx2, sy2, dst2.x, dst2.y);
	setEquations(6, sx3, sy3, dst3.x, dst3.y);

	// Gaussian elimination with partial pivoting to solve 8x8
	auto solve8 = [&](double M[8][9], double out[8]) -> bool {
		for (int col = 0; col < 8; ++col) {
			// Find pivot
			int pivot = col;
			double best = std::fabs(M[col][col]);
			for (int r = col + 1; r < 8; ++r) {
				double v = std::fabs(M[r][col]);
				if (v > best) {
					best = v;
					pivot = r;
				}
			}
			if (best < GEOMETRY_EPSILON) {
				return false;
			}

			// Swap rows
			if (pivot != col) {
				for (int c = col; c < 9; std::swap(M[col][c], M[pivot][c]), ++c)
					;
			}

			// Normalize pivot row
			double div = M[col][col];
			for (int c = col; c < 9; M[col][c] /= div, ++c)
				;

			// Eliminate other rows
			for (int r = 0; r < 8; ++r) {
				if (r == col) {
					continue;
				}
				double factor = M[r][col];
				if (std::fabs(factor) < GEOMETRY_EPSILON) {
					continue;
				}
				for (int c = col; c < 9; M[r][c] -= factor * M[col][c], ++c)
					;
			}
		}

		for (int i = 0; i < 8; out[i] = M[i][8], ++i)
			;
		return true;
	};

	double h[8] = {};
	double A_copy[8][9];
	for (int r = 0; r < 8; ++r) {
		for (int c = 0; c < 9; ++c) {
			A_copy[r][c] = A[r][c];
		}
	}

	if (!solve8(A_copy, h)) {
		// Degenerate mapping
		return;
	}

	// Homography H (src -> dst)
	double H[3][3] = {
		{ h[0], h[1], h[2] },
		{ h[3], h[4], h[5] },
		{ h[6], h[7], 1.0 },
	};

	// Invert 3x3 matrix
	auto invert3x3 = [](double M[3][3], double inv[3][3]) -> bool {
		const double a = M[0][0], b = M[0][1], c = M[0][2];
		const double d = M[1][0], e = M[1][1], f = M[1][2];
		const double g = M[2][0], h = M[2][1], i = M[2][2];

		const double A = (e * i - f * h);
		const double B = -(d * i - f * g);
		const double C = (d * h - e * g);
		const double D = -(b * i - c * h);
		const double E = (a * i - c * g);
		const double F = -(a * h - b * g);
		const double G = (b * f - c * e);
		const double Hc = -(a * f - c * d);
		const double I = (a * e - b * d);

		const double det = a * A + b * B + c * C;
		if (std::fabs(det) < GEOMETRY_EPSILON) {
			return false;
		}

		const double inv_det = 1.0 / det;
		inv[0][0] = A * inv_det;
		inv[0][1] = D * inv_det;
		inv[0][2] = G * inv_det;
		inv[1][0] = B * inv_det;
		inv[1][1] = E * inv_det;
		inv[1][2] = Hc * inv_det;
		inv[2][0] = C * inv_det;
		inv[2][1] = F * inv_det;
		inv[2][2] = I * inv_det;
		return true;
	};

	double Hinv[3][3];
	if (!invert3x3(H, Hinv)) {
		return;
	}

	// Helper: sample src with bilinear interpolation
	auto sampleBilinear = [&](double x, double y) -> Color3 {
		// Clamp to valid range
		x = std::clamp(x, 0.0, sw - 1.0);
		y = std::clamp(y, 0.0, sh - 1.0);

		int ix = static_cast<int>(std::floor(x));
		int iy = static_cast<int>(std::floor(y));
		int ix1 = std::min(ix + 1, src_image.width_ - 1);
		int iy1 = std::min(iy + 1, src_image.height_ - 1);

		double tx = x - static_cast<double>(ix);
		double ty = y - static_cast<double>(iy);

		const Color3 &c00 = src_image.image_[iy][ix];
		const Color3 &c10 = src_image.image_[iy][ix1];
		const Color3 &c01 = src_image.image_[iy1][ix];
		const Color3 &c11 = src_image.image_[iy1][ix1];

		// Assume Color3 supports + and * with scalar (common for Vec3/Color classes).
		return c00 * ((1.0 - tx) * (1.0 - ty)) + c10 * (tx * (1.0 - ty)) + c01 * ((1.0 - tx) * ty) + c11 * (tx * ty);
	};

	// Iterate only in clipped bounding box, and only pixels inside quad
	for (int y = y0; y <= y1; ++y) {
		for (int x = x0; x <= x1; ++x) {
			// Use pixel center for slightly better quality
			const double X = static_cast<double>(x) + 0.5;
			const double Y = static_cast<double>(y) + 0.5;

			if (!pointInPoly(X, Y, quad, 4)) {
				continue;
			}

			// Map destination -> source using Hinv
			const double denom = Hinv[2][0] * X + Hinv[2][1] * Y + Hinv[2][2];
			if (std::fabs(denom) < GEOMETRY_EPSILON) {
				continue;
			}

			const double src_x = (Hinv[0][0] * X + Hinv[0][1] * Y + Hinv[0][2]) / denom;
			const double src_y = (Hinv[1][0] * X + Hinv[1][1] * Y + Hinv[1][2]) / denom;

			// Only paste if mapped point is inside source image
			if (src_x < 0.0 || src_y < 0.0 || src_x > sw - 1.0 || src_y > sh - 1.0) {
				continue;
			}

			image_[y][x] = sampleBilinear(src_x, src_y);
		}
	}
}

bool Texture::save_texture(const std::string &file_path) const {
	if (!image_) {
		throw std::runtime_error("Texture is not initialized.");
	}

	// Only support PPM format for simplicity.
	if (file_path.size() < 4 || file_path.substr(file_path.size() - 4) != ".ppm") {
		return false;
	}

	FILE *fp = fopen(file_path.c_str(), "wb");
	if (!fp) {
		return false;
	}

	// Write PPM header
	fprintf(fp, "P6\n%d %d\n255\n", width_, height_);

	// Write pixel data
	for (int y = 0; y < height_; ++y) {
		for (int x = 0; x < width_; ++x) {
			const Color3 &color = image_[y][x];
			unsigned char r = static_cast<unsigned char>(std::clamp(color.x() * 255.0, 0.0, 255.0));
			unsigned char g = static_cast<unsigned char>(std::clamp(color.y() * 255.0, 0.0, 255.0));
			unsigned char b = static_cast<unsigned char>(std::clamp(color.z() * 255.0, 0.0, 255.0));
			fwrite(&r, sizeof(unsigned char), 1, fp);
			fwrite(&g, sizeof(unsigned char), 1, fp);
			fwrite(&b, sizeof(unsigned char), 1, fp);
		}
	}

	fclose(fp);
	return true;
}

}
