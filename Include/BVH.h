#pragma once
#include "MathLib.h"
#include "Primitives.h"

#include <vector>

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
	void Rebuild();
	bool Traverse(Ray& ray) const;

	Triangle GetTriangle(uint32_t index) const;
	uint32_t GetMaxDepth() const;
	BuildOption& GetBuildOption() { return m_build_option; }

private:
	struct BVHNode
	{
		AABB bounds;
		// Right child is always left_child + 1
		uint32_t left_first = 0;
		uint32_t prim_count = 0;
	};

private:
	void CalculateNodeBounds(BVHNode& node, const std::vector<uint32_t>& tri_indices);
	void Subdivide(uint32_t node_index, uint32_t depth);
	float EvaluateSAH(BVHNode& node, uint32_t axis, float split_pos);
	void Split(BVHNode& node, uint32_t axis, float split_pos, uint32_t depth);

private:
	BuildOption m_build_option = BuildOption_SAHSplitIntervals;

	std::vector<BVHNode> m_nodes;
	uint32_t m_current_node = 0;
	uint32_t m_max_depth = 0;

	std::vector<Triangle> m_triangles;
	std::vector<uint32_t> m_tri_indices;
	std::vector<Vec3> m_centroids;

};
