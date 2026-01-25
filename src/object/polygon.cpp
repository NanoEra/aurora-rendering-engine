#include <algorithm>
#include <basic/math.h>
#include <iostream>
#include <object/object_set.h>
#include <object/polygon.h>

namespace are {

Polygon::Polygon(const std::vector<Point3> &vertices, Material *material, Texture *texture) : material_(material), texture_(texture), vertices_(vertices) {
	// Initialize plane based on vertices.
	// Assumes the polygon is planar and defined by at least 3 non-collinear vertices.
	if (vertices_.size() < 3) {
		// Degenerate polygon: assign default plane to avoid undefined behavior.
		plane_.normal = Vec3(0, 1, 0);
		plane_.d = 0;
		return;
	}

	// Compute normal using the first three vertices.
	Vec3 v0 = vertices_[0];
	Vec3 v1 = vertices_[1];
	Vec3 v2 = vertices_[2];

	Vec3 u = v1 - v0;
	Vec3 v = v2 - v0;
	Vec3 n = u.cross(v);

	double len = n.length();
	if (len > GEOMETRY_EPSILON) {
		plane_.normal = n / len;
	} else {
		// Degenerate case (collinear points), try to find a valid normal or default
		plane_.normal = Vec3(0, 0, 0);
	}

	// Calculate plane constant d: N . P + d = 0 => d = -N . P
	plane_.d = -plane_.normal.dot(v0);
}

bool Polygon::point_in(const Point3 &point) const {
	if (vertices_.size() < 3) {
		return false;
	}

	// Step 1: Check if the point lies on the polygon's plane.
	double dist = plane_.normal.dot(point) + plane_.d;
	if (std::fabs(dist) > GEOMETRY_EPSILON) {
		return false;
	}

	// Step 2: Check if the point is within the polygon boundaries.
	// Using the same-side method for convex polygons.
	// The point must be on the same side of all edges defined by the polygon vertices.

	Vec3 n = plane_.normal;
	int n_verts = vertices_.size();

	// Determine the expected sign of the cross product relative to the normal.
	// We check the first edge to establish the reference side (or check all against the normal).
	// Robust approach: Check that all cross products have the same sign (positive or negative) relative to N.

	bool sign_determined = false;
	bool positive_side = false;

	for (int i = 0; i < n_verts; ++i) {
		const Point3 &curr = vertices_[i];
		const Point3 &next = vertices_[(i + 1) % n_verts];

		// Edge vector
		Vec3 edge = next - curr;
		// Vector from vertex to point
		Vec3 vec_to_p = point - curr;

		// Cross product: edge x vec_to_p.
		// The direction of this vector indicates which side of the edge the point is on.
		Vec3 cross_prod = edge.cross(vec_to_p);

		// Dot product with polygon normal to get signed distance (scaled)
		double dot_val = cross_prod.dot(n);

		// If the point is exactly on the edge (within epsilon), the dot_val is close to 0.
		// Otherwise, we check consistency of the sign.
		if (std::fabs(dot_val) > GEOMETRY_EPSILON) {
			if (!sign_determined) {
				sign_determined = true;
				positive_side = (dot_val > 0);
			} else {
				// If the current sign differs from the reference sign, point is outside.
				if ((dot_val > 0) != positive_side) {
					return false;
				}
			}
		}
	}

	return true;
}

bool Polygon::intersect_ray(const Ray &ray, Point3 &hit_point) const {
	// Step 1: Intersect ray with the infinite plane.
	// Plane equation: N . P + d = 0
	// Ray equation: P(t) = Q + t * D
	// Substitute: N . (Q + t * D) + d = 0
	// Solve for t: t = -(N . Q + d) / (N . D)

	double denom = plane_.normal.dot(ray.D);

	// Check if ray is parallel to the plane.
	if (std::fabs(denom) < GEOMETRY_EPSILON) {
		return false;
	}

	double t = -(plane_.normal.dot(ray.Q) + plane_.d) / denom;

	// Check if intersection is behind the ray origin.
	if (t < GEOMETRY_EPSILON) {
		return false;
	}

	// Calculate the candidate intersection point.
	Point3 candidate_point = ray.Q + t * ray.D;

	// Step 2: Check if the intersection point lies inside the polygon boundaries.
	if (point_in(candidate_point)) {
		hit_point = candidate_point;
		return true;
	}

	return false;
}

static int diguicishu = 0;

Texture Polygon::trace_texture(const ObjectSet &object_set, const Point3 &viewport_origin_point) const {
	// 1. 拷贝当前纹理的副本作为返回值
	Texture result(texture_->width_, texture_->height_, Color3(0, 0, 0));
	for (int y = 0; y < texture_->height_; ++y) {
		for (int x = 0; x < texture_->width_; ++x) {
			result.pixel(x, y) = texture_->pixel(x, y);
		}
	}

	// 2. 检测当前纹理是否需要处理反射
	Point3 dummy_new_origin;
	if (!material_->reflect(plane_, viewport_origin_point, dummy_new_origin) || ++diguicishu >= 10) {
		// 不需要处理反射，直接返回拷贝的副本
		return result;
	}

	// 3. 建立当前Polygon平面的局部2D坐标系
	Vec3 normal = plane_.normal.normalized();

	// 选择两个正交的基向量 u_axis 和 v_axis
	Vec3 u_axis, v_axis;
	if (std::abs(normal.x()) < 0.9) {
		u_axis = normal.cross(Vec3(1, 0, 0)).normalized();
	} else {
		u_axis = normal.cross(Vec3(0, 1, 0)).normalized();
	}
	v_axis = normal.cross(u_axis).normalized();

	// 计算当前polygon顶点在平面坐标系中的UV边界（最小包围盒）
	Point3 plane_origin = vertices_[0];
	double min_u_self = std::numeric_limits<double>::max();
	double max_u_self = std::numeric_limits<double>::lowest();
	double min_v_self = std::numeric_limits<double>::max();
	double max_v_self = std::numeric_limits<double>::lowest();

	std::vector<std::pair<double, double>> self_uvs;
	for (const auto &vertex : vertices_) {
		Vec3 rel = vertex - plane_origin;
		double u = rel.dot(u_axis);
		double v = rel.dot(v_axis);
		self_uvs.push_back({ u, v });
		min_u_self = std::min(min_u_self, u);
		max_u_self = std::max(max_u_self, u);
		min_v_self = std::min(min_v_self, v);
		max_v_self = std::max(max_v_self, v);
	}

	double width_self = max_u_self - min_u_self;
	double height_self = max_v_self - min_v_self;

	// 4. 构建视锥体边界平面（由viewport_origin_point和当前polygon的边组成）
	std::vector<Plane> frustum_planes;
	int n_verts = static_cast<int>(vertices_.size());

	// 计算当前polygon的中心点
	Point3 self_centroid(0, 0, 0);
	for (const auto &v : vertices_) {
		self_centroid += v;
	}
	self_centroid = self_centroid / static_cast<double>(n_verts);

	for (int i = 0; i < n_verts; ++i) {
		const Point3 &v1 = vertices_[i];
		const Point3 &v2 = vertices_[(i + 1) % n_verts];

		// 构建由viewport_origin_point, v1, v2三点确定的平面
		Vec3 edge = v2 - v1;
		Vec3 to_v1 = v1 - viewport_origin_point;
		Vec3 frustum_normal = edge.cross(to_v1);

		if (frustum_normal.near_zero()) {
			continue; // 退化情况，跳过
		}
		frustum_normal.normalize();

		// 确保法向量指向视锥体内部（朝向polygon中心）
		Vec3 to_centroid = self_centroid - viewport_origin_point;
		if (frustum_normal.dot(to_centroid) < 0) {
			frustum_normal = -frustum_normal;
		}

		frustum_planes.push_back(Plane(viewport_origin_point, frustum_normal));
	}

	// 添加viewport平面本身（确保物体在viewport的正面）
	Vec3 view_dir = (self_centroid - viewport_origin_point).normalized();
	Plane near_plane(viewport_origin_point, view_dir);

	// 5. 遍历object_set，找出与视锥体相交的polygon
	struct PolygonDistance {
		Polygon *polygon;
		double distance;
		Point3 centroid;
	};
	std::vector<PolygonDistance> intersected_polygons;

	for (Polygon *obj : object_set.object_set) {
		// 跳过自身
		if (obj == this) {
			continue;
		}

		// 计算目标polygon的重心
		Point3 target_centroid(0, 0, 0);
		int target_n = static_cast<int>(obj->vertices_.size());
		for (const auto &v : obj->vertices_) {
			target_centroid += v;
		}
		target_centroid = target_centroid / static_cast<double>(target_n);

		// 检测目标polygon是否与视锥体相交
		bool intersects = false;

		for (const Point3 &vertex : vertices_) { // 检查原点到当前polygon顶点的射线是否与目标polygon相交
			Vec3 direction = vertex - viewport_origin_point;
			if(direction.near_zero()) {
				continue;
			}
			Ray ray(viewport_origin_point, direction);
			Point3 hit;
			// std::cout << "Checking hit: " << vertices_.size() << std::endl;
			// 视锥体原点到viewport当前端点的连线与目标polugon相交
			if(obj->intersect_ray(ray, hit)) {
				// std::cout << "HIT!" << std::endl;
				// 检测hit_point是否在viewport前面
				double dist_to_hit = (hit - viewport_origin_point).length();
				double dist_to_vertex = (vertex - viewport_origin_point).length();
				if(dist_to_hit > dist_to_vertex - GEOMETRY_EPSILON) {
					intersects = true;
					break;
				}
			}
		}

		if (!intersects) { // 如果顶点都不在内部，检查从viewport_origin_point到目标顶点的射线是否与当前polygon相交
			for (const Point3 &vertex : obj->vertices_) {
				Vec3 dir = vertex - viewport_origin_point;
				if (dir.near_zero())
					continue;
				dir.normalize();

				Ray ray(viewport_origin_point, dir);
				Point3 hit;
				if (intersect_ray(ray, hit)) {
					// 确保目标顶点在hit点之后（相对于viewport_origin_point）
					double dist_to_hit = (hit - viewport_origin_point).length();
					double dist_to_vertex = (vertex - viewport_origin_point).length();
					if (dist_to_vertex > dist_to_hit - GEOMETRY_EPSILON) {
						intersects = true;
						break;
					}
				}
			}
		}

		if (!intersects) { // 检查目标polygon的边是否穿过视锥体
			for (int i = 0; i < target_n && !intersects; ++i) {
				const Point3 &edge_start = obj->vertices_[i];
				const Point3 &edge_end = obj->vertices_[(i + 1) % target_n];
				Vec3 edge_dir = edge_end - edge_start;
				double edge_len = edge_dir.length();
				if (edge_len < GEOMETRY_EPSILON)
					continue;
				edge_dir = edge_dir / edge_len;

				Ray edge_ray(edge_start, edge_dir);
				Point3 plane_hit;
				if (plane_.intersect_ray(edge_ray, plane_hit)) {
					double t = (plane_hit - edge_start).dot(edge_dir);
					if (t > GEOMETRY_EPSILON && t < edge_len - GEOMETRY_EPSILON) {
						// 交点在边上，检查是否在当前polygon内
						if (point_in(plane_hit)) {
							intersects = true;
						}
					}
				}
			}
		}

		if (intersects) {
			double dist = (target_centroid - viewport_origin_point).length();
			intersected_polygons.push_back({ obj, dist, target_centroid });
		}
	}

	std::cout << "Hitted" << intersected_polygons.size() << " polygons." << std::endl;

	// 6. 按距离降序排序（远的先绘制，近的后绘制覆盖）
	std::sort(intersected_polygons.begin(), intersected_polygons.end(),
		[](const PolygonDistance &a, const PolygonDistance &b) {
			return a.distance > b.distance;
		});

	int i = 0;
	// 7. 遍历排序后的polygon数组，进行纹理映射
	for (const auto &pd : intersected_polygons) {
		Polygon *target_polygon = pd.polygon;

		// 计算目标polygon每个顶点投影到当前平面的UV坐标
		std::vector<std::pair<double, double>> projected_uvs;
		bool projection_valid = true;

		for (const Point3 &vertex : target_polygon->vertices_) {
			Vec3 dir = vertex - viewport_origin_point;
			if (dir.near_zero()) {
				projection_valid = false;
				break;
			}
			dir.normalize();

			Ray ray(viewport_origin_point, dir);
			Point3 intersection;
			if (!plane_.intersect_ray(ray, intersection)) {
				projection_valid = false;
				break;
			}

			// 检查交点是否在viewport_origin_point和顶点之间的正确方向
			Vec3 to_intersection = intersection - viewport_origin_point;
			Vec3 to_vertex = vertex - viewport_origin_point;
			if (to_intersection.dot(to_vertex) < GEOMETRY_EPSILON) {
				projection_valid = false;
				break;
			}

			// 计算交点的2D UV坐标
			Vec3 rel = intersection - plane_origin;
			double u = rel.dot(u_axis);
			double v = rel.dot(v_axis);
			projected_uvs.push_back({ u, v });
		}

		if (!projection_valid || projected_uvs.size() != target_polygon->vertices_.size()) {
			continue;
		}

		// 获取反射后的viewport origin（用于递归调用）
		Point3 reflected_origin;
		if (!material_->reflect(target_polygon->plane_, viewport_origin_point, reflected_origin)) {
			continue;
		}

		// 递归获取目标polygon的纹理
		Texture target_texture = target_polygon->trace_texture(object_set, reflected_origin);
		// target_texture.save_texture(std::to_string(diguicishu) + "_" + std::to_string(i++) + ".ppm");

		// 计算目标polygon顶点在其自身纹理中的UV坐标（最小包围盒原则）
		Vec3 target_normal = target_polygon->plane_.normal.normalized();
		Vec3 target_u_axis, target_v_axis;
		if (std::abs(target_normal.x()) < 0.9) {
			target_u_axis = target_normal.cross(Vec3(1, 0, 0)).normalized();
		} else {
			target_u_axis = target_normal.cross(Vec3(0, 1, 0)).normalized();
		}
		target_v_axis = target_normal.cross(target_u_axis).normalized();

		Point3 target_plane_origin = target_polygon->vertices_[0];
		double target_min_u = std::numeric_limits<double>::max();
		double target_max_u = std::numeric_limits<double>::lowest();
		double target_min_v = std::numeric_limits<double>::max();
		double target_max_v = std::numeric_limits<double>::lowest();

		std::vector<std::pair<double, double>> target_local_uvs;
		for (const auto &vertex : target_polygon->vertices_) {
			Vec3 rel = vertex - target_plane_origin;
			double u = rel.dot(target_u_axis);
			double v = rel.dot(target_v_axis);
			target_local_uvs.push_back({ u, v });
			target_min_u = std::min(target_min_u, u);
			target_max_u = std::max(target_max_u, u);
			target_min_v = std::min(target_min_v, v);
			target_max_v = std::max(target_max_v, v);
		}

		double target_width = target_max_u - target_min_u;
		double target_height = target_max_v - target_min_v;

		if (target_width < GEOMETRY_EPSILON || target_height < GEOMETRY_EPSILON) {
			continue;
		}

		// 将目标polygon的局部UV转换为纹理坐标（0到1范围，然后映射到像素）
		std::vector<std::pair<double, double>> target_tex_coords;
		for (const auto &uv : target_local_uvs) {
			double tex_u = (uv.first - target_min_u) / target_width;
			double tex_v = (uv.second - target_min_v) / target_height;
			target_tex_coords.push_back({ tex_u, tex_v });
		}

		// 计算投影多边形的边界框
		double proj_min_u = std::numeric_limits<double>::max();
		double proj_max_u = std::numeric_limits<double>::lowest();
		double proj_min_v = std::numeric_limits<double>::max();
		double proj_max_v = std::numeric_limits<double>::lowest();
		for (const auto &uv : projected_uvs) {
			proj_min_u = std::min(proj_min_u, uv.first);
			proj_max_u = std::max(proj_max_u, uv.first);
			proj_min_v = std::min(proj_min_v, uv.second);
			proj_max_v = std::max(proj_max_v, uv.second);
		}

		// 裁剪到当前polygon的UV范围
		proj_min_u = std::max(proj_min_u, min_u_self);
		proj_max_u = std::min(proj_max_u, max_u_self);
		proj_min_v = std::max(proj_min_v, min_v_self);
		proj_max_v = std::min(proj_max_v, max_v_self);

		if (proj_min_u >= proj_max_u || proj_min_v >= proj_max_v) {
			continue;
		}

		// 遍历结果纹理的每个像素，进行逆映射
		for (int py = 0; py < result.height_; ++py) {
			for (int px = 0; px < result.width_; ++px) {
				// 将像素坐标转换为当前polygon平面的UV坐标
				double uv_u = min_u_self + (static_cast<double>(px) + 0.5) / result.width_ * width_self;
				double uv_v = min_v_self + (static_cast<double>(py) + 0.5) / result.height_ * height_self;

				// 检查该点是否在投影多边形内（使用重心坐标或射线法）
				// 使用射线法检测点是否在多边形内
				bool inside_projected = false;
				int num_projected = static_cast<int>(projected_uvs.size());
				int crossings = 0;
				for (int i = 0; i < num_projected; ++i) {
					const auto &p1 = projected_uvs[i];
					const auto &p2 = projected_uvs[(i + 1) % num_projected];

					if ((p1.second <= uv_v && p2.second > uv_v) || (p2.second <= uv_v && p1.second > uv_v)) {
						double t = (uv_v - p1.second) / (p2.second - p1.second);
						double x_intersect = p1.first + t * (p2.first - p1.first);
						if (uv_u < x_intersect) {
							crossings++;
						}
					}
				}
				inside_projected = (crossings % 2) == 1;

				if (!inside_projected) {
					continue;
				}

				// 检查是否在当前polygon内
				Point3 world_point = plane_origin + uv_u * u_axis + uv_v * v_axis;
				if (!point_in(world_point)) {
					continue;
				}

				// 使用重心坐标进行纹理映射（适用于任意凸多边形）
				// 对于三角形，使用重心坐标；对于其他多边形，使用广义重心坐标

				// 计算广义重心坐标（Mean Value Coordinates）
				std::vector<double> weights(num_projected, 0.0);
				double weight_sum = 0.0;
				bool on_vertex = false;
				int vertex_idx = -1;
				bool on_edge = false;
				int edge_idx = -1;
				double edge_t = 0.0;

				for (int i = 0; i < num_projected; ++i) {
					double dx = projected_uvs[i].first - uv_u;
					double dy = projected_uvs[i].second - uv_v;
					double dist = std::sqrt(dx * dx + dy * dy);
					if (dist < GEOMETRY_EPSILON) {
						on_vertex = true;
						vertex_idx = i;
						break;
					}
				}

				if (on_vertex) {
					// 点在顶点上
					double tex_x = target_tex_coords[vertex_idx].first * (target_texture.width_ - 1);
					double tex_y = target_tex_coords[vertex_idx].second * (target_texture.height_ - 1);
					int src_x = std::clamp(static_cast<int>(tex_x + 0.5), 0, target_texture.width_ - 1);
					int src_y = std::clamp(static_cast<int>(tex_y + 0.5), 0, target_texture.height_ - 1);
					result.pixel(px, py) = target_texture.pixel(src_x, src_y);
					continue;
				}

				// 检查是否在边上
				for (int i = 0; i < num_projected && !on_edge; ++i) {
					const auto &p1 = projected_uvs[i];
					const auto &p2 = projected_uvs[(i + 1) % num_projected];

					double edge_dx = p2.first - p1.first;
					double edge_dy = p2.second - p1.second;
					double edge_len_sq = edge_dx * edge_dx + edge_dy * edge_dy;

					if (edge_len_sq < GEOMETRY_EPSILON * GEOMETRY_EPSILON)
						continue;

					double t = ((uv_u - p1.first) * edge_dx + (uv_v - p1.second) * edge_dy) / edge_len_sq;
					if (t < 0.0 || t > 1.0)
						continue;

					double closest_x = p1.first + t * edge_dx;
					double closest_y = p1.second + t * edge_dy;
					double dist_to_edge = std::sqrt((uv_u - closest_x) * (uv_u - closest_x) + (uv_v - closest_y) * (uv_v - closest_y));

					if (dist_to_edge < GEOMETRY_EPSILON) {
						on_edge = true;
						edge_idx = i;
						edge_t = t;
					}
				}

				if (on_edge) {
					// 点在边上，线性插值
					int i1 = edge_idx;
					int i2 = (edge_idx + 1) % num_projected;
					double tex_u_interp = target_tex_coords[i1].first * (1.0 - edge_t) + target_tex_coords[i2].first * edge_t;
					double tex_v_interp = target_tex_coords[i1].second * (1.0 - edge_t) + target_tex_coords[i2].second * edge_t;

					double tex_x = tex_u_interp * (target_texture.width_ - 1);
					double tex_y = tex_v_interp * (target_texture.height_ - 1);
					int src_x = std::clamp(static_cast<int>(tex_x + 0.5), 0, target_texture.width_ - 1);
					int src_y = std::clamp(static_cast<int>(tex_y + 0.5), 0, target_texture.height_ - 1);
					result.pixel(px, py) = target_texture.pixel(src_x, src_y);
					continue;
				}

				// 使用Mean Value Coordinates计算广义重心坐标
				for (int i = 0; i < num_projected; ++i) {
					int prev = (i - 1 + num_projected) % num_projected;
					int next = (i + 1) % num_projected;

					double dx_i = projected_uvs[i].first - uv_u;
					double dy_i = projected_uvs[i].second - uv_v;
					double r_i = std::sqrt(dx_i * dx_i + dy_i * dy_i);

					double dx_prev = projected_uvs[prev].first - uv_u;
					double dy_prev = projected_uvs[prev].second - uv_v;
					double r_prev = std::sqrt(dx_prev * dx_prev + dy_prev * dy_prev);

					double dx_next = projected_uvs[next].first - uv_u;
					double dy_next = projected_uvs[next].second - uv_v;
					double r_next = std::sqrt(dx_next * dx_next + dy_next * dy_next);

					// 计算角度
					auto compute_tan_half_angle = [](double dx1, double dy1, double r1,
													  double dx2, double dy2, double r2) -> double {
						double dot = dx1 * dx2 + dy1 * dy2;
						double cross = dx1 * dy2 - dy1 * dx2;
						double cos_angle = dot / (r1 * r2);
						cos_angle = std::clamp(cos_angle, -1.0, 1.0);
						double sin_angle = cross / (r1 * r2);
						// tan(angle/2) = sin(angle) / (1 + cos(angle))
						double denom = 1.0 + cos_angle;
						if (std::abs(denom) < GEOMETRY_EPSILON) {
							return (sin_angle >= 0) ? 1e10 : -1e10;
						}
						return sin_angle / denom;
					};

					double tan_alpha_prev = compute_tan_half_angle(dx_prev, dy_prev, r_prev, dx_i, dy_i, r_i);
					double tan_alpha_next = compute_tan_half_angle(dx_i, dy_i, r_i, dx_next, dy_next, r_next);

					weights[i] = (tan_alpha_prev + tan_alpha_next) / r_i;
					weight_sum += weights[i];
				}

				if (std::abs(weight_sum) < GEOMETRY_EPSILON) {
					continue;
				}

				// 归一化权重并计算插值纹理坐标
				double interp_tex_u = 0.0;
				double interp_tex_v = 0.0;
				for (int i = 0; i < num_projected; ++i) {
					double normalized_weight = weights[i] / weight_sum;
					interp_tex_u += normalized_weight * target_tex_coords[i].first;
					interp_tex_v += normalized_weight * target_tex_coords[i].second;
				}

				// 将归一化纹理坐标转换为像素坐标
				double tex_x = interp_tex_u * (target_texture.width_ - 1);
				double tex_y = interp_tex_v * (target_texture.height_ - 1);

				// 双线性插值采样
				int x0 = static_cast<int>(std::floor(tex_x));
				int y0 = static_cast<int>(std::floor(tex_y));
				int x1 = x0 + 1;
				int y1 = y0 + 1;

				x0 = std::clamp(x0, 0, target_texture.width_ - 1);
				x1 = std::clamp(x1, 0, target_texture.width_ - 1);
				y0 = std::clamp(y0, 0, target_texture.height_ - 1);
				y1 = std::clamp(y1, 0, target_texture.height_ - 1);

				double fx = tex_x - std::floor(tex_x);
				double fy = tex_y - std::floor(tex_y);

				Color3 c00 = target_texture.pixel(x0, y0);
				Color3 c10 = target_texture.pixel(x1, y0);
				Color3 c01 = target_texture.pixel(x0, y1);
				Color3 c11 = target_texture.pixel(x1, y1);

				Color3 sampled_color = c00 * (1.0 - fx) * (1.0 - fy) + c10 * fx * (1.0 - fy) + c01 * (1.0 - fx) * fy + c11 * fx * fy;

				// 将采样的颜色写入结果纹理（金属反射直接覆盖）
				result.pixel(px, py) = sampled_color;
			}
		}
	}

	return result;
}

}
