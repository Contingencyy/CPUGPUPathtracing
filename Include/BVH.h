#pragma once
#include "MathLib.h"
#include "Primitives.h"

#include <vector>

class BVH;

class BVH
{
public:
	BVH() = default;

	void Build(const std::vector<Vertex>& vertices, const std::vector<uint32_t>& indices);
	void Traverse(Ray& ray);

	Triangle GetTriangle(uint32_t index) const;
	uint32_t GetMaxDepth() const;

private:
	struct BVHNode
	{
		AABB bounds;
		// Right child is always left_child + 1
		uint32_t left_first = 0;
		uint32_t prim_count = 0;
	};

private:
	void CalculateNodeBounds(uint32_t node_index);
	void Subdivide(uint32_t node_index, uint32_t depth);
	void Intersect(Ray& ray, uint32_t node_index);

private:
	std::vector<BVHNode> m_nodes;
	uint32_t m_current_node = 0;
	uint32_t m_max_depth = 0;

	std::vector<Triangle> m_triangles;
	std::vector<uint32_t> m_tri_indices;
	std::vector<Vec3> m_centroids;

};
