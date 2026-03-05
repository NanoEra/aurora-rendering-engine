#include "scene/mesh.h"
#include "utils/logger.h"
#include <algorithm>
#include <glad/glad.h>

namespace are {

Mesh::Mesh()
	: material_id_(0)
	, transform_(1.0f)
	, vao_(0)
	, vbo_(0)
	, ebo_(0)
	, uploaded_(false) {
}

Mesh::~Mesh() {
	release_gpu_resources();
}

void Mesh::set_vertices(const std::vector<Vertex> &vertices) {
	vertices_ = vertices;
	uploaded_ = false;
}

void Mesh::set_indices(const std::vector<uint> &indices) {
	indices_ = indices;
	uploaded_ = false;
}

void Mesh::set_material(uint material_id) {
	material_id_ = material_id;
}

void Mesh::set_transform(const Mat4 &transform) {
	transform_ = transform;
}

bool Mesh::upload_to_gpu() {
	if (uploaded_) {
		ARE_LOG_WARN("Mesh already uploaded to GPU");
		return true;
	}

	if (vertices_.empty()) {
		ARE_LOG_ERROR("Cannot upload mesh: no vertices");
		return false;
	}

	if (indices_.empty()) {
		ARE_LOG_ERROR("Cannot upload mesh: no indices");
		return false;
	}

	// Generate VAO
	glGenVertexArrays(1, &vao_);
	glBindVertexArray(vao_);

	// Generate and upload VBO
	glGenBuffers(1, &vbo_);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_);
	glBufferData(GL_ARRAY_BUFFER, vertices_.size() * sizeof(Vertex),
		vertices_.data(), GL_STATIC_DRAW);

	// Generate and upload EBO
	glGenBuffers(1, &ebo_);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_.size() * sizeof(uint),
		indices_.data(), GL_STATIC_DRAW);

	// Set vertex attributes
	// Location 0: Position
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
		(void *)offsetof(Vertex, position_));

	// Location 1: Normal
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
		(void *)offsetof(Vertex, normal_));

	// Location 2: TexCoord
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex),
		(void *)offsetof(Vertex, texcoord_));

	// Location 3: Tangent
	glEnableVertexAttribArray(3);
	glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
		(void *)offsetof(Vertex, tangent_));

	glBindVertexArray(0);

	uploaded_ = true;
	ARE_LOG_INFO("Mesh uploaded to GPU successfully");
	return true;
}

void Mesh::release_gpu_resources() {
	if (!uploaded_)
		return;

	if (vao_ != 0) {
		glDeleteVertexArrays(1, &vao_);
		vao_ = 0;
	}

	if (vbo_ != 0) {
		glDeleteBuffers(1, &vbo_);
		vbo_ = 0;
	}

	if (ebo_ != 0) {
		glDeleteBuffers(1, &ebo_);
		ebo_ = 0;
	}

	uploaded_ = false;
}

void Mesh::compute_tangents() {
	if (vertices_.empty() || indices_.empty()) {
		ARE_LOG_WARN("Cannot compute tangents: mesh is empty");
		return;
	}

	std::fill(vertices_.begin(), vertices_.end(), Vertex {});

	for (size_t i = 0; i < indices_.size(); i += 3) {
		uint i0 = indices_[i];
		uint i1 = indices_[i + 1];
		uint i2 = indices_[i + 2];

		Vertex &v0 = vertices_[i0];
		Vertex &v1 = vertices_[i1];
		Vertex &v2 = vertices_[i2];

		Vec3 pos0 = v0.position_;
		Vec3 pos1 = v1.position_;
		Vec3 pos2 = v2.position_;

		Vec2 uv0 = v0.texcoord_;
		Vec2 uv1 = v1.texcoord_;
		Vec2 uv2 = v2.texcoord_;

		Vec3 edge1 = pos1 - pos0;
		Vec3 edge2 = pos2 - pos0;
		Vec2 delta_uv1 = uv1 - uv0;
		Vec2 delta_uv2 = uv2 - uv0;

		float r = 1.0f / (delta_uv1.x * delta_uv2.y - delta_uv2.x * delta_uv1.y);
		if (std::abs(r) < 1e-6f)
			r = 1.0f;

		Vec3 tangent = (edge1 * delta_uv2.y - edge2 * delta_uv1.y) * r;
		tangent = glm::normalize(tangent - v0.normal_ * glm::dot(tangent, v0.normal_));

		v0.tangent_ += tangent;
		v1.tangent_ += tangent;
		v2.tangent_ += tangent;
	}

	for (auto &v : vertices_) {
		v.tangent_ = glm::normalize(v.tangent_);
	}

	ARE_LOG_INFO("Computed tangents for mesh");
}

} // namespace are
