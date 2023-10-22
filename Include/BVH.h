#pragma once
#include "MathLib.h"
#include "Primitives.h"

class BVH
{
public:
	enum BuildOption
	{
		BuildOption_NaiveSplit,
		BuildOption_SAHSplitIntervals,
		BuildOption_SAHSplitPrimitives,
		BuildOption_NumOptions
	};

public:
	BVH() = default;

	void Build(const std::vector<Vertex>& vertices, const std::vector<uint32_t>& indices, BuildOption build_option);
	void Rebuild(BuildOption build_option);
	bool Traverse(Ray& ray) const;

	Triangle GetTriangle(uint32_t index) const;
	uint32_t GetMaxDepth() const;
	void RenderImGui();

private:
	struct BVHNode
	{
		// Right child is always left_child + 1
		union { struct { Vec3 aabb_min; uint32_t left_first; }; __m128 aabb_min4 = {}; };
		union { struct { Vec3 aabb_max; uint32_t prim_count; }; __m128 aabb_max4 = {}; };
	};

private:
	void CalculateNodeBounds(BVHNode& node, const std::vector<uint32_t>& tri_indices);
	void Subdivide(uint32_t node_index, uint32_t depth);
	float EvaluateSAH(BVHNode& node, uint32_t axis, float split_pos);
	void Split(BVHNode& node, uint32_t axis, float split_pos, uint32_t depth);

private:
	BuildOption m_selected_build_option = BuildOption_SAHSplitIntervals;
	BuildOption m_current_build_option = BuildOption_SAHSplitIntervals;

	std::vector<BVHNode> m_nodes;
	uint32_t m_current_node = 0;
	uint32_t m_max_depth = 0;

	std::vector<Triangle> m_triangles;
	std::vector<uint32_t> m_tri_indices;
	std::vector<Vec3> m_centroids;

};
